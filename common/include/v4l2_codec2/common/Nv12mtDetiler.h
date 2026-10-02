// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef ANDROID_V4L2_CODEC2_COMMON_NV12MT_DETILER_H
#define ANDROID_V4L2_CODEC2_COMMON_NV12MT_DETILER_H

#include <stddef.h>
#include <stdint.h>

namespace android {

// Copies the |width| x |height| rectangle at (|left|, |top|) out of one plane of a
// V4L2_PIX_FMT_NV12MT buffer into the linear plane |dst|. Coordinates and |width| are in bytes,
// so for the interleaved chroma plane they cover CbCr pairs, and rows are chroma rows.
//
// NV12MT (Samsung MFC 5.x decoder output) stores 64x32 byte tiles. Each pair of tile rows is
// divided into 2x2 groups of tiles, stored in Z order, with every other group mirrored
// vertically. If the plane has an odd number of tile rows, the last one is stored linearly.
// |srcStride| must be a multiple of 128 and |srcHeight| a multiple of 32; they are the padded
// dimensions the driver reports for the plane.
//
// Returns false if the parameters describe an invalid plane or rectangle.
bool detileNv12mtPlane(const uint8_t* src, size_t srcStride, size_t srcHeight, size_t left,
                       size_t top, size_t width, size_t height, uint8_t* dst, size_t dstStride);

// Like detileNv12mtPlane() for the interleaved chroma plane, but splits the CbCr pairs into the
// planar |dstU| and |dstV|. |width| is in bytes and must be even.
bool detileNv12mtChromaToPlanar(const uint8_t* src, size_t srcStride, size_t srcHeight,
                                size_t left, size_t top, size_t width, size_t height,
                                uint8_t* dstU, size_t dstStrideU, uint8_t* dstV,
                                size_t dstStrideV);

}  // namespace android

#endif  // ANDROID_V4L2_CODEC2_COMMON_NV12MT_DETILER_H
