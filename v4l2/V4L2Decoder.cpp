// Copyright 2020 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//#define LOG_NDEBUG 0
#define ATRACE_TAG ATRACE_TAG_VIDEO
#define LOG_TAG "V4L2Decoder"

#include <v4l2_codec2/v4l2/V4L2Decoder.h>

#include <stdint.h>
#include <string.h>

#include <algorithm>
#include <vector>

#include <base/bind.h>
#include <base/files/scoped_file.h>
#include <android/hardware/graphics/common/1.0/types.h>
#include <base/memory/ptr_util.h>
#include <C2AllocatorGralloc.h>
#include <log/log.h>
#include <utils/Trace.h>

#include <v4l2_codec2/common/Common.h>
#include <v4l2_codec2/common/Fourcc.h>
#include <v4l2_codec2/common/H264NalParser.h>
#include <v4l2_codec2/common/HEVCNalParser.h>
#include <v4l2_codec2/common/Nv12mtDetiler.h>
#include <v4l2_codec2/plugin_store/DmabufHelpers.h>

namespace android {
namespace {

using ::android::hardware::graphics::common::V1_0::BufferUsage;

// Extra buffers for transmitting in the whole video pipeline.
constexpr size_t kNumExtraOutputBuffers = 4;
// Extra output buffers in tiled output mode. Decoded frames are copied out and the buffers
// queued again right away, the pipeline holds graphic blocks instead.
constexpr size_t kNumExtraTiledOutputBuffers = 2;
// VIDEO_MAX_FRAME, the most buffers a V4L2 queue can have.
constexpr size_t kMaxFimcSlots = 32;

bool waitForDRC(const C2ConstLinearBlock& input, std::optional<VideoCodec> codec) {
    C2ReadView view = input.map().get();
    const uint8_t* pos = view.data();
    // frame type takes the (2) position in first byte of VP9  uncompressed header
    const uint8_t kVP9FrameTypeMask = 0x4;
    // frame type takes the (0) position in first byte of VP8 uncompressed header
    const uint8_t kVP8FrameTypeMask = 0x1;

    switch (*codec) {
    case VideoCodec::H264: {
        H264NalParser parser(view.data(), view.capacity());
        return parser.locateIDR();
    }
    case VideoCodec::HEVC: {
        HEVCNalParser parser(view.data(), view.capacity());
        return parser.locateIDR();
    }
    // For VP8 and VP9 it is assumed that the input buffer contains a single
    // frame that is not fragmented.
    case VideoCodec::VP9:
        // 0 - key frame; 1 - interframe
        return ((pos[0] & kVP9FrameTypeMask) == 0);
    case VideoCodec::VP8:
        // 0 - key frame; 1 - interframe;
        return ((pos[0] & kVP8FrameTypeMask) == 0);
    }

    return false;
}

}  // namespace

// static
std::unique_ptr<VideoDecoder> V4L2Decoder::Create(
        uint32_t debugStreamId, const VideoCodec& codec, const size_t inputBufferSize,
        const size_t minNumOutputBuffers, GetPoolCB getPoolCb, OutputCB outputCb, ErrorCB errorCb,
        scoped_refptr<::base::SequencedTaskRunner> taskRunner, bool isSecure) {
    std::unique_ptr<V4L2Decoder> decoder =
            ::base::WrapUnique<V4L2Decoder>(new V4L2Decoder(debugStreamId, taskRunner));
    if (!decoder->start(codec, inputBufferSize, minNumOutputBuffers, std::move(getPoolCb),
                        std::move(outputCb), std::move(errorCb), isSecure)) {
        return nullptr;
    }
    return decoder;
}

V4L2Decoder::V4L2Decoder(uint32_t debugStreamId,
                         scoped_refptr<::base::SequencedTaskRunner> taskRunner)
      : mDebugStreamId(debugStreamId), mTaskRunner(std::move(taskRunner)) {
    ALOGV("%s()", __func__);

    mWeakThis = mWeakThisFactory.GetWeakPtr();
}

V4L2Decoder::~V4L2Decoder() {
    ALOGV("%s()", __func__);
    ALOG_ASSERT(mTaskRunner->RunsTasksInCurrentSequence());

    mWeakThisFactory.InvalidateWeakPtrs();

    std::queue<V4L2ReadableBufferRef>().swap(mDecodedBuffers);
    std::queue<std::pair<std::unique_ptr<VideoFrame>, uint32_t>>().swap(mFreeFrames);
    mFimc.reset();
    mOutputDmabufs.clear();

    // Streamoff input and output queue.
    if (mOutputQueue) {
        mOutputQueue->streamoff();
        mOutputQueue->deallocateBuffers();
        mOutputQueue = nullptr;
    }
    if (mInputQueue) {
        mInputQueue->streamoff();
        mInputQueue->deallocateBuffers();
        mInputQueue = nullptr;
    }
    if (mDevice) {
        mDevice->stopPolling();
        mDevice = nullptr;
    }
    if (mInitialEosBuffer) {
        mInitialEosBuffer = nullptr;
    }
}

bool V4L2Decoder::start(const VideoCodec& codec, const size_t inputBufferSize,
                        const size_t minNumOutputBuffers, GetPoolCB getPoolCb, OutputCB outputCb,
                        ErrorCB errorCb, bool isSecure) {
    ATRACE_CALL();
    ALOGV("%s(codec=%s, inputBufferSize=%zu, minNumOutputBuffers=%zu)", __func__,
          VideoCodecToString(codec), inputBufferSize, minNumOutputBuffers);
    ALOG_ASSERT(mTaskRunner->RunsTasksInCurrentSequence());

    mMinNumOutputBuffers = minNumOutputBuffers;
    mGetPoolCb = std::move(getPoolCb);
    mOutputCb = std::move(outputCb);
    mErrorCb = std::move(errorCb);
    mCodec = codec;
    mIsSecure = isSecure;

    if (mState == State::Error) {
        ALOGE("Ignore due to error state.");
        return false;
    }

    mDevice = V4L2Device::create(mDebugStreamId);

    const uint32_t inputPixelFormat = V4L2Device::videoCodecToPixFmt(codec);
    if (!mDevice->open(V4L2Device::Type::kDecoder, inputPixelFormat)) {
        ALOGE("Failed to open device for %s", VideoCodecToString(codec));
        return false;
    }

    if (!mDevice->hasCapabilities(V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING)) {
        ALOGE("Device does not have VIDEO_M2M_MPLANE and STREAMING capabilities.");
        return false;
    }

    const std::vector<uint32_t> outputFormats =
            mDevice->enumerateSupportedPixelformats(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
    const bool hasSupportedOutputFormat =
            std::any_of(outputFormats.begin(), outputFormats.end(), [](uint32_t pixfmt) {
                return std::find(kSupportedOutputFourccs.begin(), kSupportedOutputFourccs.end(),
                                 pixfmt) != kSupportedOutputFourccs.end();
            });
    mTiledOutput = !hasSupportedOutputFormat &&
                   std::find(outputFormats.begin(), outputFormats.end(), V4L2_PIX_FMT_NV12MT) !=
                           outputFormats.end();
    if (mTiledOutput) {
        if (isSecure) {
            ALOGE("Tiled output needs CPU access to the frames, cannot decode secure streams");
            return false;
        }
        mFimc = V4L2FimcConverter::Create();
        ALOGI("Device only outputs NV12MT, decoded frames are converted by %s",
              mFimc ? mFimc->name().c_str() : "the CPU");
    }

    // s5p-mfc (the tiled output device) only accepts V4L2_DEC_CMD_STOP while the input queue is
    // streaming, and then acts on it. It does support draining.
    if (!mTiledOutput && !sendV4L2DecoderCmd(false)) {
        ALOGE("Device does not support flushing (V4L2_DEC_CMD_STOP)");
        return false;
    }

    // Subscribe to the resolution change event.
    struct v4l2_event_subscription sub;
    memset(&sub, 0, sizeof(sub));
    sub.type = V4L2_EVENT_SOURCE_CHANGE;
    if (mDevice->ioctl(VIDIOC_SUBSCRIBE_EVENT, &sub) != 0) {
        ALOGE("ioctl() failed: VIDIOC_SUBSCRIBE_EVENT: V4L2_EVENT_SOURCE_CHANGE");
        return false;
    }

    // Create Input/Output V4L2Queue, and setup input queue.
    mInputQueue = mDevice->getQueue(V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
    mOutputQueue = mDevice->getQueue(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
    if (!mInputQueue || !mOutputQueue) {
        ALOGE("Failed to create V4L2 queue.");
        return false;
    }
    if (!setupInputFormat(inputPixelFormat, inputBufferSize)) {
        ALOGE("Failed to setup input format.");
        return false;
    }
    if (mTiledOutput) {
        // s5p-mfc cannot allocate output buffers before it has parsed the stream header, and does
        // not signal the initial resolution. The output is set up after the first input buffer.
        mNeedOutputSetup = true;
    } else if (!setupInitialOutput()) {
        ALOGE("Unable to setup initial output");
        return false;
    }

    if (!mDevice->startPolling(mTaskRunner,
                               ::base::BindRepeating(&V4L2Decoder::serviceDeviceTask, mWeakThis),
                               ::base::BindRepeating(&V4L2Decoder::onError, mWeakThis))) {
        ALOGE("Failed to start polling V4L2 device.");
        return false;
    }

    setState(State::Idle);
    return true;
}

bool V4L2Decoder::setupInputFormat(const uint32_t inputPixelFormat, const size_t inputBufferSize) {
    ALOGV("%s(inputPixelFormat=%u, inputBufferSize=%zu)", __func__, inputPixelFormat,
          inputBufferSize);
    ALOG_ASSERT(mTaskRunner->RunsTasksInCurrentSequence());

    // Check if the format is supported.
    std::vector<uint32_t> formats =
            mDevice->enumerateSupportedPixelformats(V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
    if (std::find(formats.begin(), formats.end(), inputPixelFormat) == formats.end()) {
        ALOGE("Input codec s not supported by device.");
        return false;
    }

    // Setup the input format.
    auto format = mInputQueue->setFormat(inputPixelFormat, ui::Size(), inputBufferSize, 0);
    if (!format) {
        ALOGE("Failed to call IOCTL to set input format.");
        return false;
    }
    ALOG_ASSERT(format->fmt.pix_mp.pixelformat == inputPixelFormat);

    // s5p-mfc only supports MMAP buffers for decoding.
    const size_t numInputBuffers = mTiledOutput ? kNumTiledInputBuffers : kNumInputBuffers;
    const enum v4l2_memory memory = mTiledOutput ? V4L2_MEMORY_MMAP : V4L2_MEMORY_DMABUF;
    if (mInputQueue->allocateBuffers(numInputBuffers, memory) == 0) {
        ALOGE("Failed to allocate input buffer.");
        return false;
    }
    if (!mInputQueue->streamon()) {
        ALOGE("Failed to streamon input queue.");
        return false;
    }
    return true;
}

bool V4L2Decoder::setupInitialOutput() {
    ATRACE_CALL();
    ALOGV("%s()", __func__);
    ALOG_ASSERT(mTaskRunner->RunsTasksInCurrentSequence());

    if (!setupMinimalOutputFormat()) {
        ALOGE("Failed to set minimal resolution for initial output buffers");
        return false;
    }

    if (!startOutputQueue(1, V4L2_MEMORY_DMABUF)) {
        ALOGE("Failed to start initialy output queue");
        return false;
    }

    std::optional<V4L2WritableBufferRef> eosBuffer = mOutputQueue->getFreeBuffer();
    if (!eosBuffer) {
        ALOGE("Failed to acquire initial EOS buffer");
        return false;
    }

    mInitialEosBuffer =
            new GraphicBuffer(mCodedSize.getWidth(), mCodedSize.getHeight(),
                              static_cast<PixelFormat>(HalPixelFormat::YCBCR_420_888),
                              GraphicBuffer::USAGE_HW_VIDEO_ENCODER, "V4L2DecodeComponent");

    if (mInitialEosBuffer->initCheck() != NO_ERROR) {
        return false;
    }

    std::vector<int> fds;
    for (size_t i = 0; i < mInitialEosBuffer->handle->numFds; i++) {
        fds.push_back(mInitialEosBuffer->handle->data[i]);
    }

    if (!std::move(*eosBuffer).queueDMABuf(fds)) {
        ALOGE("Failed to queue initial EOS buffer");
        return false;
    }

    return true;
}

bool V4L2Decoder::setupMinimalOutputFormat() {
    ui::Size minResolution, maxResolution;

    for (const uint32_t& pixfmt :
         mDevice->enumerateSupportedPixelformats(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)) {
        if (std::find(kSupportedOutputFourccs.begin(), kSupportedOutputFourccs.end(), pixfmt) ==
            kSupportedOutputFourccs.end()) {
            ALOGD("Pixel format %s is not supported, skipping...", fourccToString(pixfmt).c_str());
            continue;
        }

        mDevice->getSupportedResolution(pixfmt, &minResolution, &maxResolution);
        if (minResolution.isEmpty()) {
            minResolution.set(128, 128);
        }

        if (mOutputQueue->setFormat(pixfmt, minResolution, 0) != std::nullopt) {
            return true;
        }
    }

    ALOGE("Failed to find supported pixel format");
    return false;
}

bool V4L2Decoder::startOutputQueue(size_t minOutputBuffersCount, enum v4l2_memory memory) {
    ALOGV("%s()", __func__);
    ALOG_ASSERT(mTaskRunner->RunsTasksInCurrentSequence());

    const std::optional<struct v4l2_format> format = getFormatInfo();
    std::optional<size_t> numOutputBuffers = getNumOutputBuffers();
    if (!format || !numOutputBuffers) {
        return false;
    }
    // In tiled output mode the pipeline holds graphic blocks, not V4L2 buffers.
    if (!mTiledOutput) {
        *numOutputBuffers = std::max(*numOutputBuffers, minOutputBuffersCount);
    }

    const ui::Size codedSize(format->fmt.pix_mp.width, format->fmt.pix_mp.height);
    if (mTiledOutput) {
        // s5p-mfc refuses S_FMT while the input queue streams, and always outputs NV12MT.
        if (format->fmt.pix_mp.pixelformat != V4L2_PIX_FMT_NV12MT ||
            format->fmt.pix_mp.num_planes != 2) {
            ALOGE("Unexpected output format %s with %u planes",
                  fourccToString(format->fmt.pix_mp.pixelformat).c_str(),
                  format->fmt.pix_mp.num_planes);
            return false;
        }
        mTiledFormat = *format;
    } else if (!setupOutputFormat(codedSize)) {
        return false;
    }

    const std::optional<struct v4l2_format> adjustedFormat = getFormatInfo();
    if (!adjustedFormat) {
        return false;
    }
    mCodedSize.set(adjustedFormat->fmt.pix_mp.width, adjustedFormat->fmt.pix_mp.height);
    mVisibleRect = getVisibleRect(mCodedSize);

    ALOGI("Need %zu output buffers. coded size: %s, visible rect: %s", *numOutputBuffers,
          toString(mCodedSize).c_str(), toString(mVisibleRect).c_str());
    if (isEmpty(mCodedSize)) {
        ALOGE("Failed to get resolution from V4L2 driver.");
        return false;
    }

    std::queue<V4L2ReadableBufferRef>().swap(mDecodedBuffers);
    mOutputDmabufs.clear();
    if (mOutputQueue->isStreaming()) {
        mOutputQueue->streamoff();
    }
    if (mOutputQueue->allocatedBuffersCount() > 0) {
        mOutputQueue->deallocateBuffers();
    }

    mFrameAtDevice.clear();
    mBlockIdToV4L2Id.clear();
    while (!mReuseFrameQueue.empty()) {
        mReuseFrameQueue.pop();
    }

    const size_t adjustedNumOutputBuffers =
            mOutputQueue->allocateBuffers(*numOutputBuffers, memory);
    if (adjustedNumOutputBuffers == 0) {
        ALOGE("Failed to allocate output buffer.");
        return false;
    }

    ALOGV("Allocated %zu output buffers.", adjustedNumOutputBuffers);
    if (!mOutputQueue->streamon()) {
        ALOGE("Failed to streamon output queue.");
        return false;
    }

    if (mFimc && !exportTiledOutputBuffers()) {
        ALOGW("Cannot export the output buffers, detiling with the CPU instead of %s",
              mFimc->name().c_str());
        mFimc.reset();
        mOutputDmabufs.clear();
    }

    return true;
}

void V4L2Decoder::decode(std::unique_ptr<ConstBitstreamBuffer> buffer, DecodeCB decodeCb) {
    ATRACE_CALL();
    ALOGV("%s(id=%d)", __func__, buffer->id);
    ALOG_ASSERT(mTaskRunner->RunsTasksInCurrentSequence());

    if (mState == State::Error) {
        ALOGE("Ignore due to error state.");
        mTaskRunner->PostTask(FROM_HERE, ::base::BindOnce(std::move(decodeCb),
                                                          VideoDecoder::DecodeStatus::kError));
        return;
    }

    if (mState == State::Idle) {
        setState(State::Decoding);
    }

    // To determine if the DRC is pending, the access to the frame data is
    // required. It's not possible to access the frame directly for the secure
    // playback, so this check must be skipped. b/279834186
    if (!mIsSecure && mInitialEosBuffer && !mPendingDRC)
        mPendingDRC = waitForDRC(buffer->dmabuf, mCodec);

    mDecodeRequests.push(DecodeRequest(std::move(buffer), std::move(decodeCb)));
    pumpDecodeRequest();
}

void V4L2Decoder::drain(DecodeCB drainCb) {
    ATRACE_CALL();
    ALOGV("%s()", __func__);
    ALOG_ASSERT(mTaskRunner->RunsTasksInCurrentSequence());

    switch (mState) {
    case State::Idle:
        ALOGV("Nothing need to drain, ignore.");
        mTaskRunner->PostTask(
                FROM_HERE, ::base::BindOnce(std::move(drainCb), VideoDecoder::DecodeStatus::kOk));
        return;

    case State::Decoding:
        mDecodeRequests.push(DecodeRequest(nullptr, std::move(drainCb)));
        pumpDecodeRequest();
        return;

    case State::Draining:
    case State::Error:
        ALOGE("Ignore due to wrong state: %s", StateToString(mState));
        mTaskRunner->PostTask(FROM_HERE, ::base::BindOnce(std::move(drainCb),
                                                          VideoDecoder::DecodeStatus::kError));
        return;
    }
}

void V4L2Decoder::pumpDecodeRequest() {
    ATRACE_CALL();
    ALOGV("%s()", __func__);
    ALOG_ASSERT(mTaskRunner->RunsTasksInCurrentSequence());

    if (mState != State::Decoding) return;

    while (!mDecodeRequests.empty()) {
        // Drain the decoder.
        if (mDecodeRequests.front().buffer == nullptr) {
            ALOGV("Get drain request.");
            // Send the flush command after all input buffers are dequeued. This makes
            // sure all previous resolution changes have been handled because the
            // driver must hold the input buffer that triggers resolution change. The
            // driver cannot decode data in it without new output buffers. If we send
            // the flush now and a queued input buffer triggers resolution change
            // later, the driver will send an output buffer that has
            // V4L2_BUF_FLAG_LAST. But some queued input buffer have not been decoded
            // yet. Also, V4L2VDA calls STREAMOFF and STREAMON after resolution
            // change. They implicitly send a V4L2_DEC_CMD_STOP and V4L2_DEC_CMD_START
            // to the decoder.
            if (mInputQueue->queuedBuffersCount() > 0) {
                ALOGV("Wait for all input buffers dequeued.");
                return;
            }

            // If output queue is not streaming, then device is unable to notify
            // whenever draining is finished. (EOS frame cannot be dequeued).
            // This is likely to happen in the event of that the first resolution
            // change event wasn't dequeued before the drain request.
            if (!mOutputQueue->isStreaming()) {
                ALOGV("Wait for output queue to start streaming");
                return;
            }

            auto request = std::move(mDecodeRequests.front());
            mDecodeRequests.pop();

            // There is one more case that EOS frame cannot be dequeued because
            // the first resolution change event wasn't dequeued before - output
            // queues on the host are not streaming but ARCVM has no knowledge about
            // it. Check if first resolution change event was received and if there
            // was no previously sent non-empty frame (other than SPS/PPS/EOS) that
            // may trigger config from host side.
            // Drain can only be finished if we are sure there was no stream = no
            // single frame in the stack.
            if (mInitialEosBuffer && !mPendingDRC) {
                ALOGV("Terminate drain, because there was no stream");
                mTaskRunner->PostTask(FROM_HERE, ::base::BindOnce(std::move(request.decodeCb),
                                                                  VideoDecoder::DecodeStatus::kOk));
                return;
            }

            if (!sendV4L2DecoderCmd(false)) {
                std::move(request.decodeCb).Run(VideoDecoder::DecodeStatus::kError);
                onError();
                return;
            }
            mDrainCb = std::move(request.decodeCb);
            setState(State::Draining);
            return;
        }

        if (mTiledOutput) {
            std::optional<V4L2WritableBufferRef> inputBuffer = mInputQueue->getFreeBuffer();
            // Pause if no free input buffer. We resume decoding after dequeueing input buffers.
            if (!inputBuffer) {
                ALOGV("There is no free input buffer.");
                return;
            }

            auto request = std::move(mDecodeRequests.front());
            mDecodeRequests.pop();

            const int32_t bitstreamId = request.buffer->id;
            const size_t size = request.buffer->size;
            if (size > inputBuffer->getPlaneSize(0)) {
                ALOGE("The input size (%zu) is not enough, we need %zu",
                      inputBuffer->getPlaneSize(0), size);
                onError();
                return;
            }

            // The device only takes MMAP buffers, copy the bitstream in.
            void* dst = inputBuffer->getPlaneMapping(0);
            C2ReadView view = request.buffer->dmabuf.map().get();
            if (dst == nullptr || view.error() != C2_OK || view.capacity() < size) {
                ALOGE("%s(): Failed to map input buffer, bitstreamId=%d", __func__, bitstreamId);
                onError();
                return;
            }
            memcpy(dst, view.data(), size);

            ALOGV("QBUF to input queue, bitstreadId=%d, size=%zu", bitstreamId, size);
            inputBuffer->setTimeStamp({.tv_sec = bitstreamId});
            inputBuffer->setPlaneDataOffset(0, 0);
            inputBuffer->setPlaneBytesUsed(0, size);
            if (!std::move(*inputBuffer).queueMMap()) {
                ALOGE("%s(): Failed to QBUF to input queue, bitstreamId=%d", __func__,
                      bitstreamId);
                onError();
                return;
            }

            mPendingDecodeCbs.insert(std::make_pair(bitstreamId, std::move(request.decodeCb)));

            if (mNeedOutputSetup) {
                mNeedOutputSetup = false;
                mTaskRunner->PostTask(
                        FROM_HERE, ::base::BindOnce(&V4L2Decoder::setupTiledOutput, mWeakThis));
            }
            continue;
        }

        auto dma_buf_id = getDmabufId(mDecodeRequests.front().buffer->dmabuf.handle()->data[0]);
        if (!dma_buf_id) {
            ALOGE("Failed to get dmabuf id");
            onError();
            return;
        }

        std::optional<V4L2WritableBufferRef> inputBuffer;
        size_t targetIndex = 0;

        // If there's an existing input buffer for this dma buffer, use it.
        for (; targetIndex < mNextInputBufferId; targetIndex++) {
            if (mLastDmaBufferId[targetIndex] == dma_buf_id) {
                break;
            }
        }

        if (targetIndex < kNumInputBuffers) {
            // If we didn't find a buffer and there is an unused buffer, use that one.
            if (targetIndex == mNextInputBufferId) {
                mNextInputBufferId++;
            }

            inputBuffer = mInputQueue->getFreeBuffer(targetIndex);
        }

        // If we didn't find a reusable/unused input buffer, clobber a free one.
        if (!inputBuffer) {
            inputBuffer = mInputQueue->getFreeBuffer();
        }

        // Pause if no free input buffer. We resume decoding after dequeueing input buffers.
        if (!inputBuffer) {
            ALOGV("There is no free input buffer.");
            return;
        }

        mLastDmaBufferId[inputBuffer->bufferId()] = *dma_buf_id;

        auto request = std::move(mDecodeRequests.front());
        mDecodeRequests.pop();

        const int32_t bitstreamId = request.buffer->id;
        ALOGV("QBUF to input queue, bitstreadId=%d", bitstreamId);
        inputBuffer->setTimeStamp({.tv_sec = bitstreamId});
        size_t planeSize = inputBuffer->getPlaneSize(0);
        if (request.buffer->size > planeSize) {
            ALOGE("The input size (%zu) is not enough, we need %zu", planeSize,
                  request.buffer->size);
            onError();
            return;
        }

        ALOGV("Set bytes_used=%zu, offset=%zu", request.buffer->offset + request.buffer->size,
              request.buffer->offset);
        inputBuffer->setPlaneDataOffset(0, request.buffer->offset);
        inputBuffer->setPlaneBytesUsed(0, request.buffer->offset + request.buffer->size);
        std::vector<int> fds;
        fds.push_back(std::move(request.buffer->dmabuf.handle()->data[0]));
        if (!std::move(*inputBuffer).queueDMABuf(fds)) {
            ALOGE("%s(): Failed to QBUF to input queue, bitstreamId=%d", __func__, bitstreamId);
            onError();
            return;
        }

        mPendingDecodeCbs.insert(std::make_pair(bitstreamId, std::move(request.decodeCb)));
    }
}

void V4L2Decoder::flush() {
    ATRACE_CALL();
    ALOGV("%s()", __func__);
    ALOG_ASSERT(mTaskRunner->RunsTasksInCurrentSequence());

    // Requests still waiting for a free input buffer belong to the flushed stream too. Their
    // works are abandoned by the caller right after this returns, so abort them now: decoding
    // them later would call back for works that no longer exist.
    while (!mDecodeRequests.empty()) {
        std::move(mDecodeRequests.front().decodeCb).Run(VideoDecoder::DecodeStatus::kAborted);
        mDecodeRequests.pop();
    }

    if (mState == State::Idle) {
        ALOGV("Nothing need to flush, ignore.");
        return;
    }
    if (mState == State::Error) {
        ALOGE("Ignore due to error state.");
        return;
    }

    // Call all pending callbacks.
    for (auto& item : mPendingDecodeCbs) {
        std::move(item.second).Run(VideoDecoder::DecodeStatus::kAborted);
    }
    mPendingDecodeCbs.clear();
    if (mDrainCb) {
        std::move(mDrainCb).Run(VideoDecoder::DecodeStatus::kAborted);
    }
    mPendingDrainDone = false;

    // Streamoff both V4L2 queues to drop input and output buffers.
    const bool isOutputStreaming = mOutputQueue->isStreaming();
    mDevice->stopPolling();
    mOutputQueue->streamoff();

    // Frames decoded before the flush and still waiting for a graphic block are dropped.
    std::queue<V4L2ReadableBufferRef>().swap(mDecodedBuffers);

    // Extract currently enqueued output picture buffers to be queued later first.
    // See b/270003218 and b/297228544.
    for (auto& [v4l2Id, frame] : mFrameAtDevice) {
        // Find corresponding mapping block ID (DMABUF ID) to V4L2 buffer ID.
        // The buffer was enqueued to device therefore such mapping have to exist.
        auto blockIdIter =
                std::find_if(mBlockIdToV4L2Id.begin(), mBlockIdToV4L2Id.end(),
                             [v4l2Id = v4l2Id](const auto& el) { return el.second == v4l2Id; });

        ALOG_ASSERT(blockIdIter != mBlockIdToV4L2Id.end());
        size_t blockId = blockIdIter->first;
        mReuseFrameQueue.push(std::make_pair(blockId, std::move(frame)));
    }
    mFrameAtDevice.clear();

    mInputQueue->streamoff();

    // Streamon both V4L2 queues.
    mInputQueue->streamon();
    if (isOutputStreaming) {
        mOutputQueue->streamon();
        if (mTiledOutput && !queueTiledOutputBuffers()) {
            onError();
            return;
        }
    }

    if (mPendingResolutionChange) {
        // The input buffer that triggered it was dropped, the driver parses the header of the
        // next one.
        mPendingResolutionChange = false;
        mNeedOutputSetup = true;
    }

    // If there is no free buffer at mOutputQueue, tryFetchVideoFrame() should be triggerred after
    // a buffer is DQBUF from output queue. Now all the buffers are dropped at mOutputQueue, we
    // have to trigger tryFetchVideoFrame() here.
    if (mVideoFramePool) {
        tryFetchVideoFrame();
    }

    if (!mDevice->startPolling(mTaskRunner,
                               ::base::BindRepeating(&V4L2Decoder::serviceDeviceTask, mWeakThis),
                               ::base::BindRepeating(&V4L2Decoder::onError, mWeakThis))) {
        ALOGE("Failed to start polling V4L2 device.");
        onError();
        return;
    }

    setState(State::Idle);
}

void V4L2Decoder::serviceDeviceTask(bool event) {
    ATRACE_CALL();
    ALOGV("%s(event=%d) state=%s InputQueue(%s):%zu+%zu/%zu, OutputQueue(%s):%zu+%zu/%zu", __func__,
          event, StateToString(mState), (mInputQueue->isStreaming() ? "streamon" : "streamoff"),
          mInputQueue->freeBuffersCount(), mInputQueue->queuedBuffersCount(),
          mInputQueue->allocatedBuffersCount(),
          (mOutputQueue->isStreaming() ? "streamon" : "streamoff"),
          mOutputQueue->freeBuffersCount(), mOutputQueue->queuedBuffersCount(),
          mOutputQueue->allocatedBuffersCount());

    ALOG_ASSERT(mTaskRunner->RunsTasksInCurrentSequence());

    if (mState == State::Error) return;

    // Dequeue output and input queue.
    bool inputDequeued = false;
    while (mInputQueue->queuedBuffersCount() > 0) {
        bool success;
        V4L2ReadableBufferRef dequeuedBuffer;
        std::tie(success, dequeuedBuffer) = mInputQueue->dequeueBuffer();
        if (!success) {
            ALOGE("Failed to dequeue buffer from input queue.");
            onError();
            return;
        }
        if (!dequeuedBuffer) break;

        inputDequeued = true;

        // Run the corresponding decode callback.
        int32_t id = dequeuedBuffer->getTimeStamp().tv_sec;
        ALOGV("DQBUF from input queue, bitstreamId=%d", id);
        auto it = mPendingDecodeCbs.find(id);
        if (it == mPendingDecodeCbs.end()) {
            ALOGW("Callback is already abandoned.");
            continue;
        }
        std::move(it->second).Run(VideoDecoder::DecodeStatus::kOk);
        mPendingDecodeCbs.erase(it);
    }

    bool outputDequeued = false;
    while (mOutputQueue->queuedBuffersCount() > 0) {
        bool success;
        V4L2ReadableBufferRef dequeuedBuffer;
        std::tie(success, dequeuedBuffer) = mOutputQueue->dequeueBuffer();
        if (!success) {
            ALOGE("Failed to dequeue buffer from output queue.");
            onError();
            return;
        }
        if (!dequeuedBuffer) break;

        outputDequeued = true;

        const size_t bufferId = dequeuedBuffer->bufferId();
        const int32_t bitstreamId = static_cast<int32_t>(dequeuedBuffer->getTimeStamp().tv_sec);
        const size_t bytesUsed = dequeuedBuffer->getPlaneBytesUsed(0);
        const bool isLast = dequeuedBuffer->isLast();
        ALOGV("DQBUF from output queue, bufferId=%zu, bitstreamId=%d, bytesused=%zu, isLast=%d",
              bufferId, bitstreamId, bytesUsed, isLast);

        if (mTiledOutput) {
            if (bytesUsed > 0) {
                mDecodedBuffers.push(std::move(dequeuedBuffer));
            } else {
                // Empty buffers only mark the end of a drain or of a resolution change.
                ALOGV("Recycle empty buffer %zu back to V4L2 output queue.", bufferId);
                dequeuedBuffer.reset();
                auto outputBuffer = mOutputQueue->getFreeBuffer(bufferId);
                ALOG_ASSERT(outputBuffer, "V4L2 output queue slot %zu is not freed.", bufferId);
                if (!std::move(*outputBuffer).queueMMap()) {
                    ALOGE("%s(): Failed to recycle empty buffer to output queue.", __func__);
                    onError();
                    return;
                }
            }

            if (mDrainCb && isLast) {
                // The frames decoded before the end of the drain must reach the client first.
                if (mDecodedBuffers.empty()) {
                    finishDrain();
                    if (mState == State::Error) return;
                } else {
                    mPendingDrainDone = true;
                }
            }
            continue;
        }

        // Get the corresponding VideoFrame of the dequeued buffer.
        auto it = mFrameAtDevice.find(bufferId);
        ALOG_ASSERT(it != mFrameAtDevice.end(), "buffer %zu is not found at mFrameAtDevice",
                    bufferId);
        auto frame = std::move(it->second);
        mFrameAtDevice.erase(it);

        if (bytesUsed > 0) {
            ALOGV("Send output frame(bitstreamId=%d) to client", bitstreamId);
            frame->setBitstreamId(bitstreamId);
            frame->setVisibleRect(mVisibleRect);
            mOutputCb.Run(std::move(frame));
        } else {
            // Workaround(b/168750131): If the buffer is not enqueued before the next drain is done,
            // then the driver will fail to notify EOS. So we recycle the buffer immediately.
            ALOGV("Recycle empty buffer %zu back to V4L2 output queue.", bufferId);
            dequeuedBuffer.reset();
            auto outputBuffer = mOutputQueue->getFreeBuffer(bufferId);
            ALOG_ASSERT(outputBuffer, "V4L2 output queue slot %zu is not freed.", bufferId);

            if (!std::move(*outputBuffer).queueDMABuf(frame->getFDs())) {
                ALOGE("%s(): Failed to recycle empty buffer to output queue.", __func__);
                onError();
                return;
            }
            mFrameAtDevice.insert(std::make_pair(bufferId, std::move(frame)));
        }

        if (mDrainCb && isLast) {
            finishDrain();
        }
    }

    if (mTiledOutput && outputDequeued) {
        processDecodedBuffers();
        if (mState == State::Error) return;
    }

    // Handle resolution change event.
    if (event && dequeueResolutionChangeEvent()) {
        if (mTiledOutput && !mDecodedBuffers.empty()) {
            // Send the frames decoded at the previous resolution first.
            mPendingResolutionChange = true;
        } else if (!changeResolution()) {
            onError();
            return;
        }
    }

    // We freed some input buffers, continue handling decode requests.
    if (inputDequeued) {
        mTaskRunner->PostTask(FROM_HERE,
                              ::base::BindOnce(&V4L2Decoder::pumpDecodeRequest, mWeakThis));
    }
    // We free some output buffers, try to get VideoFrame.
    if (outputDequeued) {
        mTaskRunner->PostTask(FROM_HERE,
                              ::base::BindOnce(&V4L2Decoder::tryFetchVideoFrame, mWeakThis));
    }
}

bool V4L2Decoder::dequeueResolutionChangeEvent() {
    ATRACE_CALL();
    ALOGV("%s()", __func__);
    ALOG_ASSERT(mTaskRunner->RunsTasksInCurrentSequence());

    struct v4l2_event ev;
    memset(&ev, 0, sizeof(ev));
    while (mDevice->ioctl(VIDIOC_DQEVENT, &ev) == 0) {
        if (ev.type == V4L2_EVENT_SOURCE_CHANGE &&
            ev.u.src_change.changes & V4L2_EVENT_SRC_CH_RESOLUTION) {
            return true;
        }
    }
    return false;
}

bool V4L2Decoder::changeResolution() {
    ATRACE_CALL();
    ALOGV("%s()", __func__);
    ALOG_ASSERT(mTaskRunner->RunsTasksInCurrentSequence());

    if (mInitialEosBuffer) {
        mInitialEosBuffer = nullptr;
    }

    const enum v4l2_memory memory = mTiledOutput ? V4L2_MEMORY_MMAP : V4L2_MEMORY_DMABUF;
    if (!startOutputQueue(mMinNumOutputBuffers, memory)) {
        ALOGE("Failed to start output queue during DRC.");
        return false;
    }

    // If drain request is pending then it means that previous call to pumpDecodeRequest
    // stalled the request, bacause there was no way of notifing the component that
    // drain has finished. Send this request the drain to device.
    if (!mDecodeRequests.empty() && mDecodeRequests.front().buffer == nullptr) {
        mTaskRunner->PostTask(FROM_HERE,
                              ::base::BindOnce(&V4L2Decoder::pumpDecodeRequest, mWeakThis));
    }

    // Release the previous VideoFramePool before getting a new one to guarantee only one pool
    // exists at the same time.
    mVideoFramePool.reset();
    std::queue<std::pair<std::unique_ptr<VideoFrame>, uint32_t>>().swap(mFreeFrames);
    if (mTiledOutput) {
        if (!setupTiledPool() || !queueTiledOutputBuffers()) {
            return false;
        }
    } else {
        // Always use flexible pixel 420 format YCBCR_420_888 in Android.
        mVideoFramePool = mGetPoolCb.Run(mCodedSize, HalPixelFormat::YCBCR_420_888,
                                         mOutputQueue->allocatedBuffersCount(),
                                         static_cast<uint64_t>(BufferUsage::VIDEO_DECODER));
        if (!mVideoFramePool) {
            ALOGE("Failed to get block pool with size: %s", toString(mCodedSize).c_str());
            return false;
        }
    }

    tryFetchVideoFrame();
    return true;
}

bool V4L2Decoder::setupOutputFormat(const ui::Size& size) {
    for (const uint32_t& pixfmt :
         mDevice->enumerateSupportedPixelformats(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)) {
        if (std::find(kSupportedOutputFourccs.begin(), kSupportedOutputFourccs.end(), pixfmt) ==
            kSupportedOutputFourccs.end()) {
            ALOGD("Pixel format %s is not supported, skipping...", fourccToString(pixfmt).c_str());
            continue;
        }

        if (mOutputQueue->setFormat(pixfmt, size, 0) != std::nullopt) {
            return true;
        }
    }

    ALOGE("Failed to find supported pixel format");
    return false;
}

void V4L2Decoder::finishDrain() {
    ALOGV("All buffers are drained.");
    ALOG_ASSERT(mTaskRunner->RunsTasksInCurrentSequence());

    mPendingDrainDone = false;
    if (mTiledOutput) {
        // s5p-mfc has no V4L2_DEC_CMD_START, it leaves the finished state when streaming
        // restarts. No input is queued once the drain has been sent, and no decoded frame is
        // pending here, so nothing is lost.
        mDevice->stopPolling();
        mOutputQueue->streamoff();
        mInputQueue->streamoff();
        if (!mInputQueue->streamon() || !mOutputQueue->streamon() || !queueTiledOutputBuffers() ||
            !mDevice->startPolling(
                    mTaskRunner, ::base::BindRepeating(&V4L2Decoder::serviceDeviceTask, mWeakThis),
                    ::base::BindRepeating(&V4L2Decoder::onError, mWeakThis))) {
            ALOGE("Failed to restart streaming after drain");
            std::move(mDrainCb).Run(VideoDecoder::DecodeStatus::kError);
            onError();
            return;
        }
    } else {
        sendV4L2DecoderCmd(true);
    }
    std::move(mDrainCb).Run(VideoDecoder::DecodeStatus::kOk);
    setState(State::Idle);
}

void V4L2Decoder::tryFetchVideoFrame() {
    ATRACE_CALL();
    ALOGV("%s()", __func__);
    ALOG_ASSERT(mTaskRunner->RunsTasksInCurrentSequence());

    if (!mVideoFramePool) {
        ALOGE("mVideoFramePool is null, failed to get the instance after resolution change?");
        onError();
        return;
    }

    if (mTiledOutput) {
        // Keep one frame ready for the next decoded buffer.
        if (mFreeFrames.size() > mDecodedBuffers.size()) return;
        if (!mVideoFramePool->getVideoFrame(
                    ::base::BindOnce(&V4L2Decoder::onVideoFrameReady, mWeakThis))) {
            ALOGV("%s(): Previous callback is running, ignore.", __func__);
        }
        return;
    }

    if (mOutputQueue->freeBuffersCount() == 0) {
        ALOGV("No free V4L2 output buffers, ignore.");
        return;
    }

    if (mReuseFrameQueue.empty()) {
        if (!mVideoFramePool->getVideoFrame(
                    ::base::BindOnce(&V4L2Decoder::onVideoFrameReady, mWeakThis))) {
            ALOGV("%s(): Previous callback is running, ignore.", __func__);
        }

        return;
    }

    // Reuse output picture buffers that were abandoned after STREAMOFF first.
    // NOTE(b/270003218 and b/297228544): This avoids issues with lack of
    // ability to return all picture buffers on STREAMOFF from VDA and
    // saves on IPC with BufferQueue increasing overall responsiveness.
    uint32_t blockId = mReuseFrameQueue.front().first;
    std::unique_ptr<VideoFrame> frame = std::move(mReuseFrameQueue.front().second);
    mReuseFrameQueue.pop();

    // Avoid recursive calls
    mTaskRunner->PostTask(FROM_HERE, ::base::BindOnce(&V4L2Decoder::onVideoFrameReady, mWeakThis,
                                                      std::make_pair(std::move(frame), blockId)));
}

void V4L2Decoder::onVideoFrameReady(
        std::optional<VideoFramePool::FrameWithBlockId> frameWithBlockId) {
    ALOGV("%s()", __func__);
    ALOG_ASSERT(mTaskRunner->RunsTasksInCurrentSequence());

    if (!frameWithBlockId) {
        ALOGE("Got nullptr VideoFrame.");
        onError();
        return;
    }

    // Unwrap our arguments.
    std::unique_ptr<VideoFrame> frame;
    uint32_t blockId;
    std::tie(frame, blockId) = std::move(*frameWithBlockId);

    if (mTiledOutput) {
        mFreeFrames.push(std::make_pair(std::move(frame), blockId));
        processDecodedBuffers();
        return;
    }

    std::optional<V4L2WritableBufferRef> outputBuffer;
    // Find the V4L2 buffer that is associated with this block.
    auto iter = mBlockIdToV4L2Id.find(blockId);
    if (iter != mBlockIdToV4L2Id.end()) {
        // If we have met this block in the past, reuse the same V4L2 buffer.
        outputBuffer = mOutputQueue->getFreeBuffer(iter->second);
        if (!outputBuffer) {
            // NOTE(b/281477122): There is a bug in C2BufferQueueBlock. Its buffer queue slots
            // cache is inconsistent when MediaSync is used and a buffer with the same dmabuf id
            // can be returned twice despite being already in use by V4L2Decoder. We drop the
            // buffer here in order to prevent unwanted errors. It is safe, bacause its allocation
            // will be kept alive by the C2GraphicBlock instance.
            ALOGW("%s(): The frame have been supplied again, despite being already enqueued",
                  __func__);
            tryFetchVideoFrame();
            return;
        }
    } else if (mBlockIdToV4L2Id.size() < mOutputQueue->allocatedBuffersCount()) {
        // If this is the first time we see this block, give it the next
        // available V4L2 buffer.
        const size_t v4l2BufferId = mBlockIdToV4L2Id.size();
        mBlockIdToV4L2Id.emplace(blockId, v4l2BufferId);
        outputBuffer = mOutputQueue->getFreeBuffer(v4l2BufferId);
        ALOG_ASSERT(v4l2BufferId == outputBuffer->bufferId());
    } else {
        // If this happens, this is a bug in VideoFramePool. It should never
        // provide more blocks than we have V4L2 buffers.
        ALOGE("Got more different blocks than we have V4L2 buffers for.");
    }

    if (!outputBuffer) {
        ALOGE("V4L2 buffer not available. blockId=%u", blockId);
        onError();
        return;
    }

    uint32_t v4l2Id = outputBuffer->bufferId();
    ALOGV("QBUF to output queue, blockId=%u, V4L2Id=%u", blockId, v4l2Id);

    if (!std::move(*outputBuffer).queueDMABuf(frame->getFDs())) {
        ALOGE("%s(): Failed to QBUF to output queue, blockId=%u, V4L2Id=%u", __func__, blockId,
              v4l2Id);
        onError();
        return;
    }
    if (mFrameAtDevice.find(v4l2Id) != mFrameAtDevice.end()) {
        ALOGE("%s(): V4L2 buffer %d already enqueued.", __func__, v4l2Id);
        onError();
        return;
    }
    mFrameAtDevice.insert(std::make_pair(v4l2Id, std::move(frame)));

    tryFetchVideoFrame();
}

void V4L2Decoder::setupTiledOutput() {
    ATRACE_CALL();
    ALOGV("%s()", __func__);
    ALOG_ASSERT(mTaskRunner->RunsTasksInCurrentSequence());

    if (mState == State::Error) return;

    // The header is parsed from a queued input buffer: VIDIOC_G_FMT waits for that to finish, but
    // fails after a timeout if there is nothing to parse. A flush may have dropped the buffer.
    if (mInputQueue->queuedBuffersCount() == 0) {
        ALOGV("Input was flushed, wait for the next buffer");
        mNeedOutputSetup = true;
        return;
    }

    if (!changeResolution()) {
        ALOGE("Failed to set up output from the stream header");
        onError();
    }
}

bool V4L2Decoder::queueTiledOutputBuffers() {
    ALOG_ASSERT(mTaskRunner->RunsTasksInCurrentSequence());

    while (std::optional<V4L2WritableBufferRef> buffer = mOutputQueue->getFreeBuffer()) {
        const size_t bufferId = buffer->bufferId();
        if (!std::move(*buffer).queueMMap()) {
            ALOGE("%s(): Failed to QBUF to output queue, V4L2Id=%zu", __func__, bufferId);
            return false;
        }
    }
    return true;
}

void V4L2Decoder::processDecodedBuffers() {
    ATRACE_CALL();
    ALOGV("%s(): %zu decoded, %zu free frames", __func__, mDecodedBuffers.size(),
          mFreeFrames.size());
    ALOG_ASSERT(mTaskRunner->RunsTasksInCurrentSequence());

    if (mState == State::Error) return;

    while (!mDecodedBuffers.empty() && !mFreeFrames.empty()) {
        V4L2ReadableBufferRef buffer = std::move(mDecodedBuffers.front());
        mDecodedBuffers.pop();
        std::unique_ptr<VideoFrame> frame = std::move(mFreeFrames.front().first);
        const uint32_t blockId = mFreeFrames.front().second;
        mFreeFrames.pop();

        const bool converted = mUseFimc ? convertTiledFrame(*buffer, frame.get(), blockId)
                                        : copyTiledFrame(*buffer, frame.get());
        if (!converted) {
            onError();
            return;
        }

        // Give the buffer back right away, the device only decodes with enough of them queued.
        const size_t bufferId = buffer->bufferId();
        const int32_t bitstreamId = static_cast<int32_t>(buffer->getTimeStamp().tv_sec);
        buffer.reset();
        std::optional<V4L2WritableBufferRef> outputBuffer = mOutputQueue->getFreeBuffer(bufferId);
        if (!outputBuffer || !std::move(*outputBuffer).queueMMap()) {
            ALOGE("%s(): Failed to QBUF to output queue, V4L2Id=%zu", __func__, bufferId);
            onError();
            return;
        }

        ALOGV("Send output frame(bitstreamId=%d) to client", bitstreamId);
        frame->setBitstreamId(bitstreamId);
        frame->setVisibleRect(mUseFimc ? mFimcRect
                                       : Rect(mVisibleRect.width(), mVisibleRect.height()));
        mOutputCb.Run(std::move(frame));
    }

    if (!mDecodedBuffers.empty()) {
        tryFetchVideoFrame();
        return;
    }

    if (mPendingDrainDone && mDrainCb) {
        finishDrain();
        if (mState == State::Error) return;
    }
    if (mPendingResolutionChange) {
        mPendingResolutionChange = false;
        if (!changeResolution()) {
            onError();
        }
        return;
    }
    tryFetchVideoFrame();
}

bool V4L2Decoder::exportTiledOutputBuffers() {
    ALOG_ASSERT(mTaskRunner->RunsTasksInCurrentSequence());

    mOutputDmabufs.clear();
    mOutputDmabufs.resize(mOutputQueue->allocatedBuffersCount());
    for (size_t i = 0; i < mOutputDmabufs.size(); ++i) {
        for (size_t plane = 0; plane < mOutputDmabufs[i].size(); ++plane) {
            struct v4l2_exportbuffer expbuf = {};
            expbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            expbuf.index = i;
            expbuf.plane = plane;
            expbuf.flags = O_CLOEXEC | O_RDWR;
            if (mDevice->ioctl(VIDIOC_EXPBUF, &expbuf) != 0) {
                ALOGE("VIDIOC_EXPBUF failed for buffer %zu plane %zu", i, plane);
                return false;
            }
            mOutputDmabufs[i][plane].reset(expbuf.fd);
        }
    }
    return true;
}

bool V4L2Decoder::setupTiledPool() {
    ALOG_ASSERT(mTaskRunner->RunsTasksInCurrentSequence());

    // The frames are cropped while converting. The blocks cycle through the whole pipeline, so
    // the pool needs as many as the V4L2 queue would have needed otherwise.
    const ui::Size size((mVisibleRect.width() + 1) & ~1, (mVisibleRect.height() + 1) & ~1);
    mUseFimc = false;
    mFimcStride = 0;
    mFimcSlots.clear();

    if (mFimc) {
        // FIMC writes RGB for a surface, which the display can scan out unscaled (FIMD windows
        // have no YUV support). Only the FIMC writes the blocks. Byte buffers must be YUV.
        mFimcFourcc = V4L2_PIX_FMT_BGR32;
        mVideoFramePool =
                mGetPoolCb.Run(size, HalPixelFormat::BGRA_8888, mMinNumOutputBuffers, 0);
        if (mVideoFramePool && mVideoFramePool->isByteBufferMode()) {
            mVideoFramePool.reset();
            mFimcFourcc = V4L2_PIX_FMT_YVU420;
            mVideoFramePool = mGetPoolCb.Run(size, HalPixelFormat::YV12, mMinNumOutputBuffers, 0);
        }
        mUseFimc = mVideoFramePool != nullptr;
    }
    if (!mVideoFramePool) {
        // YV12 like the software decoders: the flexible format resolves to NV12, which minigbm
        // cannot allocate with the composer usage every output surface requests.
        mVideoFramePool = mGetPoolCb.Run(size, HalPixelFormat::YV12, mMinNumOutputBuffers,
                                         C2MemoryUsage::CPU_WRITE);
    }
    if (!mVideoFramePool) {
        ALOGE("Failed to get block pool with size: %s", toString(size).c_str());
        return false;
    }

    ALOGI("Converting %s frames to %s with %s", toString(size).c_str(),
          mUseFimc && mFimcFourcc == V4L2_PIX_FMT_BGR32 ? "BGRA_8888" : "YV12",
          mUseFimc ? mFimc->name().c_str() : "the CPU");
    return true;
}

bool V4L2Decoder::convertTiledFrame(const V4L2ReadableBuffer& buffer, VideoFrame* frame,
                                    uint32_t blockId) {
    ATRACE_CALL();

    // The Codec2 handle carries the gralloc stride, no need to map the block.
    uint32_t width, height, format, stride, generation, igbpSlot;
    uint64_t usage, igbpId;
    android::_UnwrapNativeCodec2GrallocMetadata(frame->handle(), &width, &height, &format, &usage,
                                                &stride, &generation, &igbpId, &igbpSlot);

    if (stride != mFimcStride) {
        // FIMC writes the YV12 chroma planes at exactly half the luma stride, Android aligns
        // them to 16 bytes.
        if (mFimcFourcc == V4L2_PIX_FMT_YVU420 && stride % 32 != 0) {
            ALOGE("YV12 stride %u is not supported by %s", stride, mFimc->name().c_str());
            return false;
        }

        // Android assumes BT.601 for SD and BT.709 for HD streams without colour information.
        const bool bt709 = mVisibleRect.height() >= 720;
        mFimcSlotCount = std::min(mMinNumOutputBuffers + 2, kMaxFimcSlots);
        if (!mFimc->configure(mCodedSize, mVisibleRect, mFimcFourcc, ui::Size(stride, height),
                              bt709, mOutputQueue->allocatedBuffersCount(), mFimcSlotCount,
                              &mFimcRect)) {
            ALOGE("Failed to set up %s for %ux%u frames", mFimc->name().c_str(), stride, height);
            return false;
        }
        mFimcStride = stride;
        mFimcSlots.clear();
    }

    auto slot = mFimcSlots.find(blockId);
    if (slot == mFimcSlots.end()) {
        if (mFimcSlots.size() >= mFimcSlotCount) {
            // More blocks than expected: start over, the blocks get mapped again.
            mFimcSlots.clear();
        }
        slot = mFimcSlots.emplace(blockId, mFimcSlots.size()).first;
    }

    const size_t bufferId = buffer.bufferId();
    if (bufferId >= mOutputDmabufs.size() || frame->getFDs().empty()) {
        ALOGE("No DMA-buf for output buffer %zu", bufferId);
        return false;
    }
    return mFimc->convert(bufferId, mOutputDmabufs[bufferId][0].get(),
                          mOutputDmabufs[bufferId][1].get(), slot->second, frame->getFDs()[0]);
}

bool V4L2Decoder::copyTiledFrame(const V4L2ReadableBuffer& buffer, VideoFrame* frame) {
    ATRACE_CALL();

    const uint8_t* srcY = static_cast<const uint8_t*>(buffer.getPlaneMapping(0));
    const uint8_t* srcUV = static_cast<const uint8_t*>(buffer.getPlaneMapping(1));
    if (srcY == nullptr || srcUV == nullptr) {
        ALOGE("Failed to map output buffer %zu", buffer.bufferId());
        return false;
    }

    // Both planes have the luma width in bytes. The chroma plane has half the rows, padded to
    // whole tiles again.
    const size_t srcStride = mTiledFormat.fmt.pix_mp.plane_fmt[0].bytesperline;
    const size_t srcHeightY = mTiledFormat.fmt.pix_mp.height;
    const size_t srcHeightUV = (srcHeightY / 2 + 31) & ~static_cast<size_t>(31);
    if (mTiledFormat.fmt.pix_mp.plane_fmt[0].sizeimage < srcStride * srcHeightY ||
        mTiledFormat.fmt.pix_mp.plane_fmt[1].sizeimage < srcStride * srcHeightUV) {
        ALOGE("Output planes are smaller than %zux%zu", srcStride, srcHeightY);
        return false;
    }

    C2GraphicView view = frame->map().get();
    if (view.error() != C2_OK) {
        ALOGE("Failed to map graphic block: %d", view.error());
        return false;
    }
    const C2PlanarLayout& layout = view.layout();
    uint8_t* const* data = view.data();
    if (layout.type != C2PlanarLayout::TYPE_YUV || layout.numPlanes != 3 ||
        layout.planes[C2PlanarLayout::PLANE_Y].colInc != 1) {
        ALOGE("Unsupported graphic block layout");
        return false;
    }

    // The frame is cropped to the visible rectangle.
    const size_t left = mVisibleRect.left & ~1;
    const size_t top = mVisibleRect.top & ~1;
    const size_t width = mVisibleRect.width();
    const size_t height = mVisibleRect.height();
    const size_t widthUV = (width + 1) & ~static_cast<size_t>(1);
    const size_t heightUV = (height + 1) / 2;

    const C2PlaneInfo& planeY = layout.planes[C2PlanarLayout::PLANE_Y];
    if (!detileNv12mtPlane(srcY, srcStride, srcHeightY, left, top, width, height,
                           data[C2PlanarLayout::PLANE_Y], planeY.rowInc)) {
        return false;
    }

    const C2PlaneInfo& planeU = layout.planes[C2PlanarLayout::PLANE_U];
    const C2PlaneInfo& planeV = layout.planes[C2PlanarLayout::PLANE_V];
    uint8_t* dstU = data[C2PlanarLayout::PLANE_U];
    uint8_t* dstV = data[C2PlanarLayout::PLANE_V];
    if (planeU.colInc == 2 && planeV.colInc == 2 && planeU.rowInc == planeV.rowInc &&
        dstV == dstU + 1) {
        // NV12, the source layout.
        return detileNv12mtPlane(srcUV, srcStride, srcHeightUV, left, top / 2, widthUV, heightUV,
                                 dstU, planeU.rowInc);
    }

    if (planeU.colInc == 1 && planeV.colInc == 1) {
        // Planar, e.g. YV12.
        return detileNv12mtChromaToPlanar(srcUV, srcStride, srcHeightUV, left, top / 2, widthUV,
                                          heightUV, dstU, planeU.rowInc, dstV, planeV.rowInc);
    }

    // Any other chroma layout: detile the interleaved CbCr rows, then split them.
    std::vector<uint8_t> rowsUV(widthUV * heightUV);
    if (!detileNv12mtPlane(srcUV, srcStride, srcHeightUV, left, top / 2, widthUV, heightUV,
                           rowsUV.data(), widthUV)) {
        return false;
    }
    for (size_t y = 0; y < heightUV; ++y) {
        const uint8_t* row = rowsUV.data() + y * widthUV;
        uint8_t* u = dstU + y * planeU.rowInc;
        uint8_t* v = dstV + y * planeV.rowInc;
        for (size_t x = 0; x < widthUV / 2; ++x) {
            u[x * planeU.colInc] = row[2 * x];
            v[x * planeV.colInc] = row[2 * x + 1];
        }
    }
    return true;
}

std::optional<size_t> V4L2Decoder::getNumOutputBuffers() {
    ALOGV("%s()", __func__);
    ALOG_ASSERT(mTaskRunner->RunsTasksInCurrentSequence());

    struct v4l2_control ctrl;
    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.id = V4L2_CID_MIN_BUFFERS_FOR_CAPTURE;
    if (mDevice->ioctl(VIDIOC_G_CTRL, &ctrl) != 0) {
        ALOGE("ioctl() failed: VIDIOC_G_CTRL");
        return std::nullopt;
    }
    ALOGV("%s() V4L2_CID_MIN_BUFFERS_FOR_CAPTURE returns %u", __func__, ctrl.value);

    return ctrl.value + (mTiledOutput ? kNumExtraTiledOutputBuffers : kNumExtraOutputBuffers);
}

std::optional<struct v4l2_format> V4L2Decoder::getFormatInfo() {
    ALOGV("%s()", __func__);
    ALOG_ASSERT(mTaskRunner->RunsTasksInCurrentSequence());

    struct v4l2_format format;
    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (mDevice->ioctl(VIDIOC_G_FMT, &format) != 0) {
        ALOGE("ioctl() failed: VIDIOC_G_FMT");
        return std::nullopt;
    }

    return format;
}

Rect V4L2Decoder::getVisibleRect(const ui::Size& codedSize) {
    ALOGV("%s()", __func__);
    ALOG_ASSERT(mTaskRunner->RunsTasksInCurrentSequence());

    struct v4l2_rect* visible_rect = nullptr;
    struct v4l2_selection selection_arg;
    memset(&selection_arg, 0, sizeof(selection_arg));
    selection_arg.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    selection_arg.target = V4L2_SEL_TGT_COMPOSE;

    if (mDevice->ioctl(VIDIOC_G_SELECTION, &selection_arg) == 0) {
        ALOGV("VIDIOC_G_SELECTION is supported");
        visible_rect = &selection_arg.r;
    } else {
        ALOGV("Fallback to VIDIOC_G_CROP");
        struct v4l2_crop crop_arg;
        memset(&crop_arg, 0, sizeof(crop_arg));
        crop_arg.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

        if (mDevice->ioctl(VIDIOC_G_CROP, &crop_arg) != 0) {
            ALOGW("ioctl() VIDIOC_G_CROP failed");
            return Rect(codedSize.width, codedSize.height);
        }
        visible_rect = &crop_arg.c;
    }

    Rect rect(visible_rect->left, visible_rect->top, visible_rect->left + visible_rect->width,
              visible_rect->top + visible_rect->height);
    ALOGV("visible rectangle is %s", toString(rect).c_str());
    if (!contains(Rect(codedSize.width, codedSize.height), rect)) {
        ALOGW("visible rectangle %s is not inside coded size %s", toString(rect).c_str(),
              toString(codedSize).c_str());
        return Rect(codedSize.width, codedSize.height);
    }
    if (rect.isEmpty()) {
        ALOGW("visible size is empty");
        return Rect(codedSize.width, codedSize.height);
    }

    return rect;
}

bool V4L2Decoder::sendV4L2DecoderCmd(bool start) {
    ALOGV("%s(start=%d)", __func__, start);
    ALOG_ASSERT(mTaskRunner->RunsTasksInCurrentSequence());

    struct v4l2_decoder_cmd cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.cmd = start ? V4L2_DEC_CMD_START : V4L2_DEC_CMD_STOP;
    if (mDevice->ioctl(VIDIOC_DECODER_CMD, &cmd) != 0) {
        ALOGE("ioctl() VIDIOC_DECODER_CMD failed: start=%d", start);
        return false;
    }

    return true;
}

void V4L2Decoder::onError() {
    ALOGV("%s()", __func__);
    ALOG_ASSERT(mTaskRunner->RunsTasksInCurrentSequence());

    setState(State::Error);
    mErrorCb.Run();
}

void V4L2Decoder::setState(State newState) {
    ALOGV("%s(%s)", __func__, StateToString(newState));
    ALOG_ASSERT(mTaskRunner->RunsTasksInCurrentSequence());

    if (mState == newState) return;
    if (mState == State::Error) {
        ALOGV("Already in Error state.");
        return;
    }

    switch (newState) {
    case State::Idle:
        break;
    case State::Decoding:
        break;
    case State::Draining:
        if (mState != State::Decoding) newState = State::Error;
        break;
    case State::Error:
        break;
    }

    ALOGI("Set state %s => %s", StateToString(mState), StateToString(newState));
    mState = newState;
}

// static
const char* V4L2Decoder::StateToString(State state) {
    switch (state) {
    case State::Idle:
        return "Idle";
    case State::Decoding:
        return "Decoding";
    case State::Draining:
        return "Draining";
    case State::Error:
        return "Error";
    }
}

}  // namespace android
