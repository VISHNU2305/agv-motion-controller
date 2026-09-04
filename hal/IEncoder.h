#pragma once

// Hardware Abstraction Layer: quadrature wheel encoder.
// Real implementation would count pulses via a pin-change/external interrupt.
// Sim implementation derives ticks from the virtual world's traveled distance.
class IEncoder {
public:
    virtual ~IEncoder() = default;

    virtual long getTicks() = 0;
    virtual void reset() = 0;
};