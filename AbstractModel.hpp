#pragma once

#include <iostream>
#include <stdexcept>
#include <vector>
#include <array>
#include <memory>

namespace GRANSAC
{
    typedef double VPFloat;

    // Each abstract model is made of abstract parameters
    // Could be anything from a point (that make a 2D line or 3D plane or image correspondences) to a line
    class AbstractParameter
    {
    public:
	virtual ~AbstractParameter(void) {}; // To make this polymorphic we add dummy destructor
    };

    // Abstract model type for generic RANSAC model fitting
    template <int t_NumParams> /* Minimum number of parameters required to define this model*/
    class AbstractModel
    {
    protected:
	std::array<std::shared_ptr<AbstractParameter>, t_NumParams> m_MinModelParams;


    // This take the votes are returns the average of the votes;
	virtual std::vector<GRANSAC::VPFloat> ComputeDistanceMeasure(std::vector<std::shared_ptr<AbstractParameter>>  votes) = 0;

    public:
    std::pair<std::vector<GRANSAC::VPFloat>,Point2f> m_model;
    //Initialize set's the value for the above mode;
	virtual void Initialize(std::vector<std::shared_ptr<AbstractParameter>> InputParams) = 0;

	//Evaluate get's the array of votes for the given set's of Parameters obtained.
	virtual std::vector<VPFloat> Evaluate(std::vector<std::shared_ptr<AbstractParameter>> EvaluateParams) = 0;

	virtual std::array<std::shared_ptr<AbstractParameter>, t_NumParams> GetModelParams(void) { return m_MinModelParams; };
    };
} // namespace GRANSAC
