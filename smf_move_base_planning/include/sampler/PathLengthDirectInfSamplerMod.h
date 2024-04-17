/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, University of Toronto
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the University of Toronto nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: Jonathan Gammell */

#ifndef OMPL_BASE_SAMPLERS_INFORMED_PATH_LENGTH_DIRECT_INFORMED_SAMPLER_MOD
#define OMPL_BASE_SAMPLERS_INFORMED_PATH_LENGTH_DIRECT_INFORMED_SAMPLER_MOD

// We inherit from InformedStateSampler
#include "ompl/base/samplers/InformedStateSampler.h"
#include <ompl/base/Planner.h>
// For std::list
#include <list>

namespace ompl
{
    namespace base
    {
        class PathLengthDirectInfSamplerMod : public InformedSampler
        {
        public:
            PathLengthDirectInfSamplerMod(const ProblemDefinitionPtr &probDefn, unsigned int maxNumberCalls, const StateSpacePtr space, const PlannerPtr &planner,
                                          const std::vector<State *> solution_start_states);
            ~PathLengthDirectInfSamplerMod() override;

            bool sampleUniform(State *statePtr, const Cost &maxCost) override;

            bool sampleUniform(State *statePtr, const Cost &minCost, const Cost &maxCost) override;

            bool hasInformedMeasure() const override;

            double getInformedMeasure(const Cost &currentCost) const override;

            Cost heuristicSolnCost(const State *statePtr) const override;

            void getNextSample(State *state);

        private:
            using ProlateHyperspheroidCPtr = std::shared_ptr<const ompl::ProlateHyperspheroid>;

            // Helper functions:
            // High level
            bool sampleUniform(State *statePtr, const Cost &maxCost, unsigned int *iters);

            bool sampleBoundsRejectPhs(State *statePtr, unsigned int *iters);

            bool samplePhsRejectBounds(State *statePtr, unsigned int *iters);

            // Low level
            std::vector<double> getInformedSubstate(const State *statePtr) const;

            void createFullState(State *statePtr, const std::vector<double> &informedVector);

            void updatePhsDefinitions(const Cost &maxCost);

            ompl::ProlateHyperspheroidPtr randomPhsPtr();

            bool keepSample(const std::vector<double> &informedVector);

            bool isInAnyPhs(const std::vector<double> &informedVector) const;

            bool isInPhs(const ProlateHyperspheroidCPtr &phsCPtr, const std::vector<double> &informedVector) const;

            unsigned int numberOfPhsInclusions(const std::vector<double> &informedVector) const;

            // Variables
            std::list<ompl::ProlateHyperspheroidPtr> listPhsPtrs_;

            double summedMeasure_;

            unsigned int informedIdx_;

            StateSpacePtr informedSubSpace_;

            unsigned int uninformedIdx_;

            StateSpacePtr uninformedSubSpace_;

            StateSamplerPtr baseSampler_;

            StateSamplerPtr uninformedSubSampler_;

            std::vector<State *> solution_start_states_;

            RNG rng_;
        }; // PathLengthDirectInfSamplerMod
    }
}

#endif // OMPL_BASE_SAMPLERS_INFORMED_DIRECT_PATH_LENGTH_INFORMED_SAMPLER_