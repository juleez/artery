# mcm-twolane

Mixed traffic on a two-lane motorway: more lane changes than in `mcm-highway`, and not every
vehicle is connected. A vehicle either runs the cooperative MCM service or sends no V2X
messages at all.

## Road and traffic

- straight motorway, 4 km, two lanes of 3.5 m, 120 km/h, on-ramp at 1 km with a 300 m
  acceleration lane
- Poisson arrivals, about 2300 veh/h on the mainline and 380 veh/h on the ramp at density 1.0,
  desired speeds spread by 15 % (cars) so that faster vehicles overtake
- vehicle categories, encoded in the SUMO id (flow id) and equipped by `services.xml`:

| Category | Share | Equipment | Behaviour |
|---|---|---|---|
| `coop_*` | 60 % | CA, `McService` | automated, yields to requested trajectories |
| `manual_*` | 26 % | none | human driven (`sigma=0.5`), lane changes by SUMO's sublane model |
| `truck_*` | 13 % | none | human driven trucks at 80 km/h, reason for overtaking |

All nodes are `artery.envmod.Car` with front and rear radars (`sensors.xml`): `McService`
takes vehicles without MCM from its sensors into account when it checks a lane change for
conflicts (constant velocity prediction).

Regenerate the network after editing the node or edge files:

    netconvert -c mcm-twolane.netccfg

## Configurations

As in `mcm-highway`: `General`, `sumo-gui`, `ground-truth`, `density`, `negotiation` and
`legitimate-deviations`. The output files are the same as well.
