#ifndef ARTERY_MCM_NONCOOPERATIVEMCSERVICE_H_
#define ARTERY_MCM_NONCOOPERATIVEMCSERVICE_H_

#include "artery/mcm/McService.h"

namespace artery
{

/**
 * Connected, honest vehicle which shares its plans and requests cooperation for
 * its own lane changes, but never yields to requested trajectories of others
 * (TR 103 578: the "cost" of every cooperation is too high). With explicit
 * responses it answers with a NACK.
 */
class NonCooperativeMcService : public McService
{
protected:
    bool isCooperative() const override { return false; }
};

} // namespace artery

#endif /* ARTERY_MCM_NONCOOPERATIVEMCSERVICE_H_ */
