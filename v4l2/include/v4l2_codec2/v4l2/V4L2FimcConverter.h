// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef ANDROID_V4L2_CODEC2_V4L2_V4L2_FIMC_CONVERTER_H
#define ANDROID_V4L2_CODEC2_V4L2_V4L2_FIMC_CONVERTER_H

#include <stddef.h>
#include <stdint.h>

#include <memory>
#include <string>

#include <base/files/scoped_file.h>
#include <ui/Rect.h>
#include <ui/Size.h>

namespace android {

// Converts the V4L2_PIX_FMT_NV12MT frames of the Samsung MFC 5.x decoder with an Exynos FIMC
// mem-to-mem device, from DMA-buf to DMA-buf. The conversion is synchronous.
class V4L2FimcConverter {
public:
    // Opens the FIMC m2m video node named by the vendor.v4l2_codec2.fimc property ("fimc.2.m2m"
    // by default, "none" disables it). Returns nullptr if it is disabled or cannot be opened.
    static std::unique_ptr<V4L2FimcConverter> Create();
    ~V4L2FimcConverter();

    // Sets up conversion of NV12MT frames of |codedSize|, cropped to |crop|, into |dstFourcc|
    // (V4L2_PIX_FMT_BGR32 or V4L2_PIX_FMT_YVU420) frames of |dstSize|, whose width is the line
    // stride in pixels. The crop is written unscaled to the top left of the destination.
    // |bt709| selects the ITU-R BT.709 matrix, else BT.601; the source is limited range.
    // |numInputs| and |numOutputs| are the number of distinct buffers used on each side.
    // FIMC aligns the written rectangle (width to 16 pixels), so |written| receives the part of
    // the crop actually written, at the top left of the destination.
    bool configure(const ui::Size& codedSize, const Rect& crop, uint32_t dstFourcc,
                   const ui::Size& dstSize, bool bt709, size_t numInputs, size_t numOutputs,
                   Rect* written);

    // Converts input |inputId| (NV12MT luma and chroma DMA-bufs) into output |outputId|
    // (DMA-buf |outputFd|). The same id must always be used with the same DMA-bufs, so the
    // device does not map them again. Returns once the frame has been written.
    bool convert(size_t inputId, int lumaFd, int chromaFd, size_t outputId, int outputFd);

    const std::string& name() const { return mName; }

private:
    V4L2FimcConverter(::base::ScopedFD fd, std::string name);
    bool dequeue(uint32_t type);
    void stop();

    ::base::ScopedFD mFd;
    const std::string mName;
    bool mStreaming = false;
    // Payload of the two NV12MT source planes.
    uint32_t mSrcPlaneSizes[2] = {};
};

}  // namespace android

#endif  // ANDROID_V4L2_CODEC2_V4L2_V4L2_FIMC_CONVERTER_H
