# mcm-highway

Scenario for the trajectory misbehavior detector (thesis Chapter 6).

## Road and traffic

- straight motorway, 4 km, three lanes of 3.5 m, 120 km/h
- one on-ramp at 1 km with a 300 m acceleration lane (`hwAcc`), which forces merges
- Poisson arrivals: 5400 veh/h on the mainline, 900 veh/h on the ramp at density 1.0
  (`--scale` in `omnetpp.ini`), 85 % cars with spread desired speed, 15 % trucks
- all vehicles are automated (`sigma=0`) and equipped with CA service and `McService`
- continuous lane changes (`lanechange.duration=3` in the sumocfg): SUMO moves the vehicle
  laterally at constant speed, which is what the honest service announces

The straight road keeps curvature out of the error measures; the ramp and the speed
spread produce strategic merges, speed gain and keep right lane changes. A warm-up of
about 130 s is needed until the vehicles fill the road.

Regenerate the network after editing the node or edge files:

    netconvert -c mcm-highway.netccfg

## Configurations

| Config | Purpose |
|---|---|
| `General` | honest traffic at density 1.0 |
| `sumo-gui` | the same with SUMO GUI |
| `ground-truth` | honest traffic, SUMO writes FCD (driven trajectories) next to the MCM logs |
| `density` | honest traffic at density 0.5, 1.0 and 1.5 |
| `legitimate-deviations` | sweep of abort probability, abort variant (with/without cancellation) and drop probability |

## Output

Besides the usual `.sca` files, `McService` writes CSV files to `results/`:

- `<config>-<run>-stations.csv`: station id, SUMO id and service type (ground truth for attackers)
- `<config>-<run>-maneuvers.csv`: one line per maneuver with role, type, timing and outcome
- `<config>-<run>-mcm-tx.csv`: every transmitted MCM including its trajectory points
  (`timeOffset:deltaEast:deltaNorth:speed:heading` in ms, cm, cm, cm/s, 0.1 degree)

`analysis/announcement_error.py` compares the announced trajectories with the FCD output of
the `ground-truth` config and prints the honest lateral and longitudinal error distribution:

    python3 analysis/announcement_error.py results ground-truth-0
