#ifndef ARTERY_MCM_MCSERVICE_H_
#define ARTERY_MCM_MCSERVICE_H_

#include "artery/mcm/BaseMcService.h"

namespace artery
{

/**
 * Honest maneuver coordination service.
 *
 * Besides driving what it announces, it produces the legitimate deviations of
 * thesis Section 5.3: with probability abortProbability a committed lane change
 * is aborted at a random point of the lateral movement and the vehicle returns
 * into its origin lane, either announcing the return with a new reference
 * trajectory (variant A) or silently (variant B). Merges out of an ending lane
 * are not aborted.
 */
class McService : public BaseMcService
{
protected:
    void initialize() override;
    Execution decideExecution(const Maneuver&) override;

private:
    double mAbortProbability = 0.0;
    bool mAbortAnnounced = true;
};

} // namespace artery

#endif /* ARTERY_MCM_MCSERVICE_H_ */
