// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef OPENCV_QUALITYBASE_HPP
#define OPENCV_QUALITYBASE_HPP

#include <vector>
#include <opencv2/core.hpp>

/**
@defgroup quality Image Quality Analysis (IQA) API
*/

namespace cv
{
namespace quality
{

//! @addtogroup quality
//! @{

/************************************ Quality Base Class ************************************/

class CV_EXPORTS_W QualityBase
    : public virtual Algorithm
{
public:

    /** @brief Destructor */
    virtual ~QualityBase() = default;

    /**
    @brief Compute quality score per channel with the per-channel score in each element of the resulting cv::Scalar.  See specific algorithm for interpreting result scores
    @param cmpImgs comparison image(s), or image(s) to evalute for no-reference quality algorithms
    */
    virtual CV_WRAP cv::Scalar compute( InputArrayOfArrays cmpImgs ) = 0;

    /** @brief Returns output quality map images that were generated during computation, if supported by the algorithm  */
    virtual CV_WRAP void getQualityMaps(OutputArrayOfArrays dst) const
    {
        if (!dst.needed() || _qualityMaps.empty() )
            return;

        auto qMaps = InputArray(_qualityMaps);
        dst.create(qMaps.size(), qMaps.type());
        dst.assign(_qualityMaps);
    }

    /** @brief Implements Algorithm::clear()  */
    CV_WRAP void clear() CV_OVERRIDE { _qualityMaps.clear(); Algorithm::clear(); }

    /** @brief Implements Algorithm::empty()  */
    CV_WRAP bool empty() const CV_OVERRIDE { return _qualityMaps.empty(); }

protected:

    /** @brief internal quality map type default */
    using _quality_map_type = cv::UMat;

    /** @brief Output quality maps if generated by algorithm */
    std::vector<_quality_map_type> _qualityMaps;

};  // QualityBase
//! @}
}   // quality
}   // cv
#endif