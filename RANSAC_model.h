#ifndef RANSAC_MODEL_H

#define RANSAC_MODEL_H

#include <opencv2/core/core.hpp>
#include "AbstractModel.hpp"
#include <limits>
namespace cmt
{

class Point2D
    : public GRANSAC::AbstractParameter
{
public:
    Point2D(Point2f p_points,int p_class)
    {
	m_normalized = p_points;
	m_class = p_class;
    };

    Point2f m_normalized;
    int m_class;
};

class HomographyModel : public GRANSAC::AbstractModel<4>
{
protected:

    vector<Point2f> m_points_normalized;
    vector<GRANSAC::VPFloat> m_votes;

    //Must be precedded with a call to the Initialize to initialize the m_model.
    virtual std::vector<GRANSAC::VPFloat> ComputeDistanceMeasure(std::vector<std::shared_ptr<GRANSAC::AbstractParameter>>  EvaluateParams) override
    {
    m_votes.clear();
    std::vector<GRANSAC::VPFloat> model = m_model.first;
    Point2f vt = m_model.second;

    for(auto& Param_ : EvaluateParams)
    {
    auto Param = std::dynamic_pointer_cast<Point2D>(Param_);
    if(Param == nullptr)
	    throw std::runtime_error("Homography::ComputeDistanceMeasure() - Passed parameter are not of type Point2D.");

    Point2f vote_ = (Param->m_normalized - model[0] * rotate(m_points_normalized[Param->m_class], model[1]));
    m_votes.push_back(norm(vote_ - vt));

    }

    return m_votes;
    };

public:
    GRANSAC::VPFloat scale,rotation;
    Point2f votes;
    HomographyModel(std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> InputParams)
    {
//    Initialize(InputParams);

    };
    //Initialize Model
    virtual void Initialize(std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> InputParams) override
    {
    votes = Point2f(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());

    if(InputParams.size() < 4)
	    throw std::runtime_error("Homography - Number of input parameters does not match minimum number required for this model.");

	//std::copy(InputParams.begin(), InputParams.end(), m_MinModelParams.begin());

    auto Point1 = std::dynamic_pointer_cast<Point2D>(InputParams[0]);
    auto Point2 = std::dynamic_pointer_cast<Point2D>(InputParams[1]);

    if(Point1 == nullptr || Point2 == nullptr)
	    throw std::runtime_error("Homogrpahy - InputParams type mismatch. It is not a Point2D.");

    std::vector<GRANSAC::VPFloat> model;

    Point2f v = Point2->m_normalized - Point1->m_normalized;
    if (norm(v) < 1 || (Point2->m_class == Point1->m_class) )
    {
        // Points to similar to create a vote for;
        m_model = std::make_pair(model,votes);
        return;
    }
//
//    Point2f h = -(m_points_normalized[Point2->m_class] - m_points_normalized[Point1->m_class]);
    bool estimate_rotation = true;
    bool estimate_scale = true;
    vector<float> changes_scale;
    if (estimate_scale) changes_scale.reserve(InputParams.size()*InputParams.size());
    vector<float> changes_angles;
    if (estimate_rotation) changes_angles.reserve(InputParams.size()*InputParams.size());

    for (size_t i = 0; i < InputParams.size(); i++)
    {
    auto Point1 = std::dynamic_pointer_cast<Point2D>(InputParams[i]);

        for (size_t j = 0; j < InputParams.size(); j++)
        {
        auto Point2 = std::dynamic_pointer_cast<Point2D>(InputParams[j]);
            if (Point2->m_class != Point1->m_class)
            {
                Point2f v = Point2->m_normalized - Point1->m_normalized;
                Point2f h = (m_points_normalized[Point2->m_class] - m_points_normalized[Point1->m_class]);
                if (estimate_scale)
                {
                    float distance = norm(v);
                    float distance_original = norm(h);
                    float change_scale = distance / distance_original;
                    changes_scale.push_back(change_scale);
                }

                if (estimate_rotation)
                {
                    float angle = atan2(v.y,v.x);
                    float angle_original = atan2(h.y,h.x);
                    float change_angle = angle - angle_original;

                    //Fix long way angles
                    if (fabs(change_angle) > M_PI) {
                        change_angle = sgn(change_angle) * 2 * M_PI + change_angle;
                    }

                    changes_angles.push_back(change_angle);
                }
            }

        }

    }

    //Do not use changes_scale, changes_angle after this point as their order is changed by median()
    if (changes_scale.size() < 2) scale = 1;
    else scale = median(changes_scale);

    if (changes_angles.size() < 2) rotation = 0;
    else rotation = median(changes_angles);
    //square root comptation
//    GRANSAC::VPFloat pol_a = h.y * v.x - h.x*v.y;
//    GRANSAC::VPFloat pol_b = -2*(h.x*v.x + h.y*v.y );
//    GRANSAC::VPFloat pol_c = (h.x*v.y) + (h.y*v.x);
//
//    GRANSAC::VPFloat determinant = pol_b*pol_b - 4*pol_a*pol_c;
//
//    GRANSAC::VPFloat x1, x2;
//    GRANSAC::VPFloat alpha_1, alpha_2, s_1, s_2;
//
//    if (determinant > 0)
//    {
//        x1 = (-pol_b + sqrt(determinant))/(2*pol_a);
//        x2 = (-pol_b - sqrt(determinant))/(2*pol_a);
//    }
//    else if(determinant == 0)
//    {
//        x1 = (-pol_b + sqrt(determinant))/(2*pol_a);
//        x2 = x1;
//    }
//    else
//    {
//        m_model = std::make_pair(model,votes);
//        return;
//    }
//
//    alpha_1 = 2*atan(x1); //Is this equivalent to the area that is mentioned earlier.
//    alpha_2 = 2*atan(x2);
//    s_1 = (v.x + (v.x*(x1*x1))) / (h.x - h.x*(x1*x1) - h.y*2*x1) ;
//    s_2 = (v.x + (v.x*(x2*x2))) / (h.x - h.x*(x2*x2) - h.y*2*x2) ;
//
//    if ( s_1 == s_2 )
//    {
//        if (s_1 < 0)
//        {
//            scale = -s_1;
//            rotation = 3.14159265;
//        }
//        else
//        {
//            scale = s_1;
//            rotation = alpha_1;
//        }
//    }
//    else
//    {
//    if (s_1 < 0)
//    {
//        scale = s_2;
//        rotation = alpha_2;
//    }
//    else
//    {
//        scale = s_1;
//        rotation = alpha_1;
//    }
//    }
    model.push_back(scale);
    model.push_back(rotation);

//    auto Point1 = std::dynamic_pointer_cast<Point2D>(InputParams[0]);
    votes = Point1->m_normalized - model[0] * rotate(m_points_normalized[Point1->m_class], model[1]);

    m_model = std::make_pair(model,votes);

    };




    virtual std::vector<GRANSAC::VPFloat> Evaluate (std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> EvaluateParams)
    {

        std::vector<GRANSAC::VPFloat> p;

        if (m_model.first.size() != 0)
            p =  ComputeDistanceMeasure(EvaluateParams);

        return p;
    };

    void set_initial_keypoints(const vector<Point2f> & points_normalized)
    {

    m_points_normalized = points_normalized;

    }


};
}
#endif
