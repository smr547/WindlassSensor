## Mechanical layout

The anchor is attached to a length of heavy chain. The other end of the chain is connected to the boat at a point called the bitter end.
The anchor is normally stowed on the bowsprit mechanically held in place by a flange at the tip of the bowprint and by tension placed on the chain by the lifting mechanism.

The lifting mechanism is known as the windlass (or anchor winch or capstan).
The windlass is comprised of a electric motor (with power supply, reversing relays and a push button control unit). The motor drives a shaft which rotates a sprocketed cog (know as a gypsy)
which engages with the anchor chain and moves the chain (and anchor) up or down depending on the direction of rotation of the motor. 

While the motor and gypsy combination is normally used for lowering the 
anchor, a mechanical clutch on the windlass can be released to allow the gypsy to rotate freely, independent of the windlass motor. This allows a deck hand to rapidly deploy the anchor 
in an emergency situation where motorised deployment migth be too slow.

A typical windlass design does not allow manual lifting of the anchor and the windlass motor must be used.

## Windlass Control

The windlass has two control lines

* UP -- causes the motor to rotate in a direction intended to lift the anchor
* DOWN -- causes the motor to rotate in the opposite a direction causing the anchor to be lowered (or deployed)

Typically these control lines operate at the working voltage of boat and supply only a small current (a few mA) sufficient to trigger the reversing relays into action. 
The contactors within the reversing relays provide the large current required by the windlass motor.

## Gypsy sensor

A magnetic reed switch or Hall Effect transistor is located in the windlass housing just below the outer rim of the gypsy. A magnet embedded in the gypsy circumference causes a pulse
as it passes this sensor. The Gypsy sensor produces one pulse on every full rotation of the gypsy.

## Chain counter 
The primary purpose of a chain counter is to report the length of chain deployed at any time. This is achieved by counting the pulses from the Gypsy Sensor while monitoring the state of the
two control lines (to determine the anchor's direction of motion).

The chain counter may also report:

* State of anchor: ``Homed``, ``Deployed`` or ``in motion`` (lifting, lowering or free falling)
* Percentage of chain deployed
* Speed of anchor deployment (m/sec) and/or time remaining before chain hits the bottom
* Scope of chain deployed (ratio of ``length of chain deployed`` to ``depth``)

The chain counter may also report events of interest to an "anchor watch" system such as:

* Deploying chain
* Retrieving chain
* Anchor on bottom
* Anchor weighed off bottom

Finally a chain counter may be equipped with remote ``UP`` and ``DOWN`` buttons allowing crew at the helm to control the 
deployment or lifting of the anchor. Typically these buttons operate in parallel with the corresponding buttons on the controller
located next to the windlass.




