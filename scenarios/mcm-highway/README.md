# mcm-highway

Scenario for the trajectory misbehavior detector (thesis Chapter 6).

## Road and traffic

- straight motorway, 4 km, three lanes of 3.5 m, 120 km/h
- one on-ramp at 1 km with a 300 m acceleration lane (`hwAcc`), which forces merges
- Poisson arrivals: 2400 veh/h on the mainline, 400 veh/h on the ramp at density 1.0
  (`--scale` in `omnetpp.ini`), cars with spread desired speed (no trucks)
- all vehicles are automated (`sigma=0`) and equipped with CA service and `McService`
- SUMO's sublane model (`lateral-resolution` in the sumocfg): `McService` drives the lateral
  movement of every lane change itself, S-shaped (quintic) with a duration drawn from
  `laneChangeDuration` (default uniform 1 to 3 s), and can abort it at any time

The straight road keeps curvature out of the error measures; the ramp and the speed
spread produce strategic merges, speed gain and keep right lane changes. A warm-up of
about 130 s is needed until the vehicles fill the road.

Regenerate the network after editing the node or edge files:

    netconvert -c mcm-highway.netccfg

## Maneuver coordination

Concept A1 of ETSI TR 103 578 (see `src/artery/mcm/BaseMcService.h`): every MCM carries the
reference trajectory; a lane change in conflict with the plans of others is sent as requested
trajectory, vehicles in conflict yield by an adapted reference trajectory (base concept) and
optionally answer with ACK/NACK (`explicitResponses`).

## Configurations

| Config | Purpose |
|---|---|
| `General` | honest traffic at density 1.0 |
| `sumo-gui` | the same with SUMO GUI and the announced trajectories as arrows |
| `ground-truth` | honest traffic, SUMO writes FCD (driven trajectories) next to the MCM logs |
| `density` | honest traffic at density 0.5, 1.0 and 1.5 |
| `negotiation` | implicit answers (base concept) against explicit ACK/NACK, density 1.0 and 1.5 |
| `lane-change-duration` | fixed lane change durations of 1, 2 and 3 s, quintic and linear shape |
| `legitimate-deviations` | sweep of abort probability, announced/silent abort and drop probability |

## Output

Besides the usual `.sca` files, `McService` writes CSV files to `results/`:

- `<config>-<run>-stations.csv`: station id, SUMO id and service type (ground truth for attackers)
- `<config>-<run>-maneuvers.csv`: one line per manoeuvre (requester) or cooperation (yielder)
  with partners, timing, outcome, answers, lane change duration and abort time
- `<config>-<run>-mcm-tx.csv`: every transmitted MCM with reference and requested trajectory
  (`timeOffset:deltaEast:deltaNorth:speed:heading` in ms, cm, cm, cm/s, 0.1 degree)
  and explicit responses (`station:trajectory:accept:reason`)

`analysis/announcement_error.py` compares the announced reference trajectories with the FCD
output of the `ground-truth` config and prints the honest lateral and longitudinal error
distribution:

    python3 analysis/announcement_error.py results ground-truth-0
