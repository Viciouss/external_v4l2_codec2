// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//#define LOG_NDEBUG 0
#define LOG_TAG "Nv12mtDetiler"

#include <v4l2_codec2/common/Nv12mtDetiler.h>

#include <string.h>

#include <algorithm>
#include <vector>

#include <libyuv/planar_functions.h>
#include <log/log.h>

namespace android {
namespace {

constexpr size_t kTileWidth = 64;
constexpr size_t kTileHeight = 32;
constexpr size_t kTileSize = kTileWidth * kTileHeight;

// Returns the position of tile (|tx|, |ty|) in memory, counted in tiles.
size_t tileIndex(size_t tx, size_t ty, size_t xTiles, size_t yTiles) {
    // With an odd number of tile rows, the last row is stored linearly.
    if ((yTiles & 1) && ty == yTiles - 1) return ty * xTiles + tx;

    // Tile row pairs are made of 2x2 groups: Z order in even groups, mirrored Z in odd groups.
    const size_t group = tx / 2;
    const size_t row = (ty & 1) ^ (group & 1);
    return (ty & ~static_cast<size_t>(1)) * xTiles + group * 4 + row * 2 + (tx & 1);
}

}  // namespace

bool detileNv12mtPlane(const uint8_t* src, size_t srcStride, size_t srcHeight, size_t left,
                       size_t top, size_t width, size_t height, uint8_t* dst, size_t dstStride) {
    if (srcStride == 0 || srcStride % (2 * kTileWidth) != 0 || srcHeight == 0 ||
        srcHeight % kTileHeight != 0) {
        ALOGE("Invalid NV12MT plane %zux%zu", srcStride, srcHeight);
        return false;
    }
    if (left + width > srcStride || top + height > srcHeight || dstStride < width) {
        ALOGE("Rectangle %zux%zu at (%zu, %zu) does not fit NV12MT plane %zux%zu", width, height,
              left, top, srcStride, srcHeight);
        return false;
    }

    const size_t xTiles = srcStride / kTileWidth;
    const size_t yTiles = srcHeight / kTileHeight;
    const size_t right = left + width;
    const size_t bottom = top + height;

    // Walk the tiles rather than the destination rows, so that each tile is read sequentially.
    for (size_t ty = top / kTileHeight; ty * kTileHeight < bottom; ++ty) {
        const size_t y0 = std::max(ty * kTileHeight, top);
        const size_t y1 = std::min((ty + 1) * kTileHeight, bottom);

        for (size_t tx = left / kTileWidth; tx * kTileWidth < right; ++tx) {
            const size_t x0 = std::max(tx * kTileWidth, left);
            const size_t x1 = std::min((tx + 1) * kTileWidth, right);

            const uint8_t* tileRow = src + tileIndex(tx, ty, xTiles, yTiles) * kTileSize +
                                     (y0 - ty * kTileHeight) * kTileWidth +
                                     (x0 - tx * kTileWidth);
            uint8_t* dstRow = dst + (y0 - top) * dstStride + (x0 - left);

            if (x1 - x0 == kTileWidth) {
                // Constant size lets the compiler inline the copy.
                for (size_t y = y0; y < y1; ++y) {
                    memcpy(dstRow, tileRow, kTileWidth);
                    tileRow += kTileWidth;
                    dstRow += dstStride;
                }
            } else {
                for (size_t y = y0; y < y1; ++y) {
                    memcpy(dstRow, tileRow, x1 - x0);
                    tileRow += kTileWidth;
                    dstRow += dstStride;
                }
            }
        }
    }

    return true;
}

bool detileNv12mtChromaToPlanar(const uint8_t* src, size_t srcStride, size_t srcHeight,
                                size_t left, size_t top, size_t width, size_t height,
                                uint8_t* dstU, size_t dstStrideU, uint8_t* dstV,
                                size_t dstStrideV) {
    if (width % 2 != 0) {
        ALOGE("Odd chroma width %zu", width);
        return false;
    }

    std::vector<uint8_t> interleaved(width * height);
    if (!detileNv12mtPlane(src, srcStride, srcHeight, left, top, width, height,
                           interleaved.data(), width)) {
        return false;
    }
    libyuv::SplitUVPlane(interleaved.data(), width, dstU, dstStrideU, dstV, dstStrideV, width / 2,
                         height);
    return true;
}

}  // namespace android
