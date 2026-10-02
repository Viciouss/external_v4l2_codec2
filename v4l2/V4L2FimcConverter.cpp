// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//#define LOG_NDEBUG 0
#define ATRACE_TAG ATRACE_TAG_VIDEO
#define LOG_TAG "V4L2FimcConverter"

#include <v4l2_codec2/v4l2/V4L2FimcConverter.h>

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <string.h>
#include <sys/ioctl.h>

#include <android-base/file.h>
#include <android-base/properties.h>
#include <android-base/strings.h>
#include <log/log.h>
#include <utils/Trace.h>

namespace android {
namespace {

// A conversion takes about 14 ms at 1920x1080.
constexpr int kConvertTimeoutMs = 500;

int xioctl(int fd, unsigned long request, void* arg) {
    int ret;
    do {
        ret = ioctl(fd, request, arg);
    } while (ret != 0 && errno == EINTR);
    return ret;
}

}  // namespace

// static
std::unique_ptr<V4L2FimcConverter> V4L2FimcConverter::Create() {
    const std::string wanted =
            ::android::base::GetProperty("vendor.v4l2_codec2.fimc", "fimc.2.m2m");
    if (wanted.empty() || wanted == "none") {
        ALOGI("FIMC conversion disabled");
        return nullptr;
    }

    std::unique_ptr<DIR, int (*)(DIR*)> dir(opendir("/sys/class/video4linux"), closedir);
    if (!dir) {
        ALOGE("Cannot list video4linux devices: %s", strerror(errno));
        return nullptr;
    }
    while (struct dirent* entry = readdir(dir.get())) {
        if (strncmp(entry->d_name, "video", 5) != 0) continue;

        std::string name;
        if (!::android::base::ReadFileToString(
                    std::string("/sys/class/video4linux/") + entry->d_name + "/name", &name)) {
            continue;
        }
        if (::android::base::Trim(name) != wanted) continue;

        const std::string path = std::string("/dev/") + entry->d_name;
        ::base::ScopedFD fd(open(path.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC));
        if (!fd.is_valid()) {
            ALOGE("Cannot open %s (%s): %s", path.c_str(), wanted.c_str(), strerror(errno));
            return nullptr;
        }
        ALOGI("Using %s (%s) for NV12MT conversion", path.c_str(), wanted.c_str());
        return std::unique_ptr<V4L2FimcConverter>(
                new V4L2FimcConverter(std::move(fd), wanted));
    }

    ALOGW("No %s video device", wanted.c_str());
    return nullptr;
}

V4L2FimcConverter::V4L2FimcConverter(::base::ScopedFD fd, std::string name)
      : mFd(std::move(fd)), mName(std::move(name)) {}

V4L2FimcConverter::~V4L2FimcConverter() {
    stop();
}

void V4L2FimcConverter::stop() {
    for (uint32_t type :
         {V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE}) {
        int arg = type;
        if (mStreaming) xioctl(mFd.get(), VIDIOC_STREAMOFF, &arg);

        struct v4l2_requestbuffers reqbufs = {};
        reqbufs.type = type;
        reqbufs.memory = V4L2_MEMORY_DMABUF;
        xioctl(mFd.get(), VIDIOC_REQBUFS, &reqbufs);
    }
    mStreaming = false;
}

bool V4L2FimcConverter::configure(const ui::Size& codedSize, const Rect& crop, uint32_t dstFourcc,
                                  const ui::Size& dstSize, bool bt709, size_t numInputs,
                                  size_t numOutputs, Rect* written) {
    ATRACE_CALL();
    ALOGV("%s(coded %dx%d, crop %d,%d %dx%d, dst %dx%d, bt709=%d)", __func__, codedSize.width,
          codedSize.height, crop.left, crop.top, crop.width(), crop.height(), dstSize.width,
          dstSize.height, bt709);
    const int fd = mFd.get();
    stop();

    // The kernel picks the conversion matrix and range from the YCbCr side. The V4L2 core
    // resets both to their defaults unless the colorspace is set too.
    const uint32_t colorspace = bt709 ? V4L2_COLORSPACE_REC709 : V4L2_COLORSPACE_SMPTE170M;
    const uint8_t ycbcrEnc = bt709 ? V4L2_YCBCR_ENC_709 : V4L2_YCBCR_ENC_601;

    struct v4l2_format src = {};
    src.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    src.fmt.pix_mp.width = codedSize.width;
    src.fmt.pix_mp.height = codedSize.height;
    src.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12MT;
    src.fmt.pix_mp.num_planes = 2;
    src.fmt.pix_mp.colorspace = colorspace;
    src.fmt.pix_mp.ycbcr_enc = ycbcrEnc;
    src.fmt.pix_mp.quantization = V4L2_QUANTIZATION_LIM_RANGE;
    if (xioctl(fd, VIDIOC_S_FMT, &src) != 0 || src.fmt.pix_mp.num_planes != 2 ||
        src.fmt.pix_mp.width != static_cast<uint32_t>(codedSize.width)) {
        ALOGE("Cannot set NV12MT %dx%d input: %s", codedSize.width, codedSize.height,
              strerror(errno));
        return false;
    }
    mSrcPlaneSizes[0] = src.fmt.pix_mp.plane_fmt[0].sizeimage;
    mSrcPlaneSizes[1] = src.fmt.pix_mp.plane_fmt[1].sizeimage;

    struct v4l2_selection sel = {};
    sel.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    sel.target = V4L2_SEL_TGT_CROP;
    sel.r = {.left = crop.left, .top = crop.top, .width = static_cast<uint32_t>(crop.width()),
             .height = static_cast<uint32_t>(crop.height())};
    if (xioctl(fd, VIDIOC_S_SELECTION, &sel) != 0) {
        ALOGE("Cannot crop the input to %d,%d %dx%d: %s", crop.left, crop.top, crop.width(),
              crop.height(), strerror(errno));
        return false;
    }

    struct v4l2_format dst = {};
    dst.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    dst.fmt.pix_mp.width = dstSize.width;
    dst.fmt.pix_mp.height = dstSize.height;
    dst.fmt.pix_mp.pixelformat = dstFourcc;
    dst.fmt.pix_mp.num_planes = 1;
    dst.fmt.pix_mp.colorspace = colorspace;
    dst.fmt.pix_mp.ycbcr_enc = ycbcrEnc;
    dst.fmt.pix_mp.quantization = V4L2_QUANTIZATION_LIM_RANGE;
    if (xioctl(fd, VIDIOC_S_FMT, &dst) != 0 || dst.fmt.pix_mp.pixelformat != dstFourcc ||
        dst.fmt.pix_mp.width != static_cast<uint32_t>(dstSize.width) ||
        dst.fmt.pix_mp.height != static_cast<uint32_t>(dstSize.height)) {
        ALOGE("Cannot set %dx%d output (got %ux%u): %s", dstSize.width, dstSize.height,
              dst.fmt.pix_mp.width, dst.fmt.pix_mp.height, strerror(errno));
        return false;
    }

    // Write the crop unscaled into the top left of the (wider) destination.
    sel = {};
    sel.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    sel.target = V4L2_SEL_TGT_COMPOSE;
    sel.r = {.left = 0, .top = 0, .width = static_cast<uint32_t>(crop.width()),
             .height = static_cast<uint32_t>(crop.height())};
    if (xioctl(fd, VIDIOC_S_SELECTION, &sel) != 0) {
        ALOGE("Cannot compose the output to %dx%d: %s", crop.width(), crop.height(),
              strerror(errno));
        return false;
    }
    if (sel.r.width != static_cast<uint32_t>(crop.width()) ||
        sel.r.height != static_cast<uint32_t>(crop.height())) {
        // The rectangle was aligned. Crop the input to the same size rather than scale it.
        const uint32_t width = sel.r.width;
        const uint32_t height = sel.r.height;
        ALOGI("Output rectangle aligned to %ux%u, cropping the input to match", width, height);
        sel = {};
        sel.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        sel.target = V4L2_SEL_TGT_CROP;
        sel.r = {.left = crop.left, .top = crop.top, .width = width, .height = height};
        if (xioctl(fd, VIDIOC_S_SELECTION, &sel) != 0 || sel.r.width != width ||
            sel.r.height != height) {
            ALOGE("Cannot crop the input to %ux%u: %s", width, height, strerror(errno));
            return false;
        }
    }
    *written = Rect(sel.r.width, sel.r.height);

    if (dstFourcc == V4L2_PIX_FMT_BGR32) {
        // The byte FIMC calls alpha would otherwise be 0, a transparent BGRA_8888 frame.
        struct v4l2_control alpha = {.id = V4L2_CID_ALPHA_COMPONENT, .value = 255};
        if (xioctl(fd, VIDIOC_S_CTRL, &alpha) != 0) {
            ALOGE("Cannot set the alpha component: %s", strerror(errno));
            return false;
        }
    }

    for (auto [type, count] : {std::pair<uint32_t, size_t>(V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
                                                          numInputs),
                               std::pair<uint32_t, size_t>(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
                                                          numOutputs)}) {
        struct v4l2_requestbuffers reqbufs = {};
        reqbufs.type = type;
        reqbufs.memory = V4L2_MEMORY_DMABUF;
        reqbufs.count = count;
        if (xioctl(fd, VIDIOC_REQBUFS, &reqbufs) != 0 || reqbufs.count < count) {
            ALOGE("Cannot get %zu buffers (got %u): %s", count, reqbufs.count, strerror(errno));
            return false;
        }
    }

    for (uint32_t type :
         {V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE}) {
        int arg = type;
        if (xioctl(fd, VIDIOC_STREAMON, &arg) != 0) {
            ALOGE("STREAMON failed: %s", strerror(errno));
            return false;
        }
        mStreaming = true;
    }
    return true;
}

bool V4L2FimcConverter::convert(size_t inputId, int lumaFd, int chromaFd, size_t outputId,
                                int outputFd) {
    ATRACE_CALL();
    const int fd = mFd.get();

    // vb2 checks bytesused against the length given here, not against the DMA-buf size.
    struct v4l2_plane srcPlanes[VIDEO_MAX_PLANES] = {};
    srcPlanes[0].m.fd = lumaFd;
    srcPlanes[0].bytesused = mSrcPlaneSizes[0];
    srcPlanes[0].length = mSrcPlaneSizes[0];
    srcPlanes[1].m.fd = chromaFd;
    srcPlanes[1].bytesused = mSrcPlaneSizes[1];
    srcPlanes[1].length = mSrcPlaneSizes[1];
    struct v4l2_buffer src = {};
    src.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    src.memory = V4L2_MEMORY_DMABUF;
    src.index = inputId;
    src.m.planes = srcPlanes;
    src.length = 2;
    if (xioctl(fd, VIDIOC_QBUF, &src) != 0) {
        ALOGE("QBUF input %zu failed: %s", inputId, strerror(errno));
        return false;
    }

    struct v4l2_plane dstPlanes[VIDEO_MAX_PLANES] = {};
    dstPlanes[0].m.fd = outputFd;
    struct v4l2_buffer dst = {};
    dst.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    dst.memory = V4L2_MEMORY_DMABUF;
    dst.index = outputId;
    dst.m.planes = dstPlanes;
    dst.length = 1;
    if (xioctl(fd, VIDIOC_QBUF, &dst) != 0) {
        ALOGE("QBUF output %zu failed: %s", outputId, strerror(errno));
        // Take the input back, the device cannot run without an output.
        stop();
        return false;
    }

    return dequeue(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) &&
           dequeue(V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
}

bool V4L2FimcConverter::dequeue(uint32_t type) {
    const int fd = mFd.get();
    struct v4l2_plane planes[VIDEO_MAX_PLANES] = {};
    struct v4l2_buffer buffer = {};
    buffer.type = type;
    buffer.memory = V4L2_MEMORY_DMABUF;
    buffer.m.planes = planes;
    buffer.length = VIDEO_MAX_PLANES;

    while (xioctl(fd, VIDIOC_DQBUF, &buffer) != 0) {
        if (errno != EAGAIN) {
            ALOGE("DQBUF failed: %s", strerror(errno));
            return false;
        }
        struct pollfd pfd = {.fd = fd, .events = POLLIN | POLLOUT};
        int ret;
        do {
            ret = poll(&pfd, 1, kConvertTimeoutMs);
        } while (ret < 0 && errno == EINTR);
        if (ret == 0) {
            ALOGE("Conversion timed out");
            return false;
        }
        if (ret < 0 || (pfd.revents & POLLERR)) {
            ALOGE("poll failed: %s", ret < 0 ? strerror(errno) : "POLLERR");
            return false;
        }
    }
    if (buffer.flags & V4L2_BUF_FLAG_ERROR) {
        ALOGE("Conversion failed");
        return false;
    }
    return true;
}

}  // namespace android
