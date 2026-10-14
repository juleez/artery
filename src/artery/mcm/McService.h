#ifndef ARTERY_MCM_MCSERVICE_H_
#define ARTERY_MCM_MCSERVICE_H_

#include "artery/mcm/BaseMcService.h"

namespace artery
{

/**
 * Honest maneuver coordination service.
 *
 * Besides driving what it announces, it produces the legitimate deviations of
 * thesis Section 5.3: with probability abortProbability an accepted lane change
 * is aborted, either with a cancellation MCM (variant A) or silently (variant B).
 * The abort happens at the announced start of the lateral movement, since SUMO
 * cannot reverse a continuous lane change once it has begun.
 */
class McService : public BaseMcService
{
protected:
    void initialize() override;
    Execution decideExecution(const Maneuver&) override;

private:
    double mAbortProbability = 0.0;
    bool mAbortWithCancel = true;
};

} // namespace artery

#endif /* ARTERY_MCM_MCSERVICE_H_ */
