# V/Hz Protection (Over-Flux / Under-Flux)

The SFC V/Hz protection provides a simple flux supervision mechanism based on the relationship:

$$
\phi \propto \frac{V}{\omega}
$$

In practice, the protection monitors a *per-unit flux proxy* and raises warnings or trips if the estimated flux is too high (over-flux) or too low (under-flux).

## Inputs and Output

### Inputs

- `voltage_magnitude_pu` (per-unit): RMS terminal voltage magnitude.
- `frequency_pu` (per-unit): electrical frequency in per-unit.

### Output (internal observable)

- `vhz_ratio_pu` (per-unit): the computed flux proxy.

## Per-Unit Flux Proxy

The protection computes:

$$
\phi_{pu} = \frac{V_{pu}}{f_{pu}}
$$

Where:

- $V_{pu}$ is the per-unit RMS voltage.
- $f_{pu}$ is the per-unit electrical frequency input (`frequency_pu`).

Rated operating point ($V_{pu}=1$, $f_{pu}=1$) gives $\phi_{pu}=1$.
Over-flux corresponds to $\phi_{pu} > 1$.
Under-flux corresponds to $\phi_{pu} < 1$.

## Enable / Disable Behavior (Low-Speed Guard)

At very low frequency, $f_{pu}$ approaches zero and $V_{pu}/f_{pu}$ becomes ill-conditioned.

To prevent false trips and divide-by-zero behavior, the protection disables itself when:

- `abs(frequency_pu) < minimum_frequency_threshold`

When disabled:

- `vhz_ratio_pu` is forced to 0.
- Over-flux and under-flux protections are not evaluated/updated.

## Over-Flux vs Under-Flux Detection

The protection evaluates the computed `vhz_ratio_pu` against two independent threshold protections:

- **Over-flux**: triggers when $\phi_{pu}$ exceeds its configured thresholds.
- **Under-flux**: triggers when $\phi_{pu}$ falls below its configured thresholds.

Each side supports:

- a warning threshold + warning delay (non-latching)
- a trip threshold + trip delay (latching)
- hysteresis on the trip threshold to define a reset boundary

## Notes on Interpretation

- Over-flux typically indicates excessive voltage for the present speed (risk of core saturation).
- Under-flux typically indicates insufficient voltage for the present speed (risk of torque capability loss and/or poor commutation margin in an LCI context).

The V/Hz protection is not a substitute for dedicated voltage, current, or commutation protections; it is a complementary, physics-based supervision signal.
