# Wiring: Filament Respooler

There is no microcontroller. 5–6 V comes in through a power rocker and feeds a low-voltage knob PWM speed module. The PWM output drives the 130-size gear motor (rated up to 6 V, 300:1 gearbox) through a resettable PTC fuse and the auto-stop switch. A "thread" button is wired in parallel with the auto-stop switch.

## Schematic

```text
  5 V 2 A (or 6 V) adapter
      |
  [DC JACK 5.5x2.1, centre +]          (-Y back wall of electronics box)
     +|            -|
  [ROCKER KCD1]     |                  (-X end wall)
      |             |
   IN+|          IN-|
  +---+-------------+---+
  | LOW-VOLTAGE PWM     |              (knob through -Y back wall)
  | SPEED MODULE        |
  +---+-------------+---+
  OUT+|         OUT-|
      |             |
  [PTC 0.75 A hold] |                  (floor cradle in the box)
      |             |
      +------+      |
      |      |      |
  [AUTO-STOP |      |                  (wiper station, COM-NO, closed
   SWITCH]  [THREAD |                   while filament holds the lever)
   COM-NO   BUTTON] |                  (momentary NO, -X end wall)
      |      |      |
      +------+      |
      |             |
      +----( M )----+                  130 motor, <= 6 V (gearbox)
```

In words:

```text
DC jack + → rocker → PWM IN+                 DC jack − → PWM IN−
PWM OUT+ → PTC → [ auto-stop NO contact  ∥  thread button ] → motor terminal A
PWM OUT− → motor terminal B
```

- **Auto-stop switch (KW11-3Z):** use the **COM** and **NO** terminals. Filament presses the lever, so the contact is closed while filament is present.
- **Thread button:** a 12 mm momentary push button (normally open) wired across the same two wires as the auto-stop switch. Hold it to jog the motor while you thread new filament past the lever. The PTC still protects the motor while the button is held.
- **Rocker:** the main power switch. It sits ahead of the PWM module. It is also how you reset a tripped PTC (see below).
- **Direction:** the take-up spool must wind the filament that arrives from −X. If the spool turns the wrong way, **swap the two motor leads** at the PWM OUT terminals.

## How the machine stops

There is no slip clutch, so the machine stops in one of two ways:

1. **Free tail (normal case):** the filament end leaves the supply spool and passes the auto-stop lever. The lever drops, the NO contact opens and the motor stops.
2. **Anchored tail:** the filament end is fixed to the supply spool core, so it never passes the lever. The filament goes tight, the motor stalls and draws about 1.5–2.5 A. Within a few seconds the PTC heats and trips to high resistance, which cuts the motor current to a few tens of mA. The motor stays stopped.

**Reset:** switch the rocker **off for about 30 s** so the PTC cools. Cut or free the tail before you switch back on. If you hold the thread button with the filament still tight, the PTC will trip again.

## PTC sizing

- **Starting part:** a radial PTC resettable fuse, about **0.75 A hold and 1.5 A trip** (for example Littelfuse 60V075, Bourns MF-R075 or RXEF075). An SMD 1206 PolySwitch soldered to a short lead also works.
- **Running current:** normal running should stay below about 0.6 A, and stall current at 6 V is about 1.5–2.5 A.
- **Bench tuning:** run a full spool. If the PTC trips during normal winding, especially when warm, at start-up or at high knob settings, use the next size up. If it takes more than about 10 s to trip on a hard stall, use the next size down. Try holds of 0.5, 0.65, 0.75, 0.9 and 1.1 A.
- **Placement:** the PTC sits in the small floor cradle in the box at X 80, Y −58. Heat-shrink its leads and leave it free of glue so it can shed heat consistently.
- **No glass fuse:** I dropped the inline glass fuse. A 5 V 2 A USB/barrel adapter limits and shuts down its own output on a short, and the PTC covers the motor branch. If you use an unprotected bench supply, add a T2A fuse in the + lead before the rocker.

## Parts

| Item | Spec | Qty |
| --- | --- | --- |
| Power supply | 5 V 2 A (USB-C PD-less or barrel) or 6 V 2 A, 5.5×2.1 mm plug, centre positive. A USB-C supply needs a USB-C to 5.5×2.1 barrel cable | 1 |
| DC panel jack | 5.5×2.1 mm, Ø11 mm hole, with nut | 1 |
| Rocker switch | KCD1 SPST, 13×19 mm snap-in | 1 |
| PWM speed module | Low-voltage knob PWM DC motor controller, input range covering 5–6 V (for example 1803BK 1.8–15 V 2 A, or a 3–35 V 5 A board). PCB about 35×25 mm, pot with M7 bushing and nut | 1 |
| PTC resettable fuse | Radial, about 0.75 A hold and 1.5 A trip (60V075 / MF-R075 / RXEF075). Buy a few sizes from 0.5 to 1.1 A for tuning | 1 (+ spares) |
| Thread button | 12 mm panel momentary, NO, ≥ 2 A (it carries stall current until the PTC trips) | 1 |
| Auto-stop microswitch | KW11-3Z / SS-5GL lever (supplied by the wiper workstream) | 1 |
| Wire, supply side | 20 AWG (0.5 mm²), red and black | ~0.3 m |
| Wire, motor and switch runs | 22 AWG (0.35 mm²) two-core. Avoid thinner wire, because at 5 V wire drop costs speed | ~1.5 m |
| Heat-shrink, crimp spade terminals (2.8 mm for the rocker, 4.8 mm for the microswitch as needed) | | |

## Routing

- Wires run on the **underside of the base**, inside the open rib cells. They cross the ribs through 10 × 6 mm notches cut in the rib bottoms. Hold them in place with tape or a dab of hot glue. Pass-through holes (Ø10) go from the top to the underside at three places:
  - **Electronics box:** X 55, Y −62, through the box floor and the base.
  - **Motor:** X 315, Y 5, just +Y of the gearbox. A 5 mm top groove runs along X 335 from the gearbox face (Y −24) to Y 5, then to the hole.
  - **Auto-stop:** X 235, Y 45, under the wiper station foot. A 5 mm top groove runs along Y 45 from X 235 to X 265 so the wires can leave the station foot anywhere along it.
- **Underside path:** from the box hole, go +Y through the rib at Y −55, then +X along the Y −35 cell row, crossing both joint blocks. Branch +Y at X 235 to reach the auto-stop hole. Continue to X 315, then go +Y to reach the motor hole.
- Leave about 100 mm of slack at the base joint so the two halves can be separated.

## Notes and risks

- **Motor voltage:** never feed the motor more than 6 V. Do not use a 12 V adapter.
- **Low-speed torque:** at 5 V with the knob low, a 130 motor can stall below about 30% duty. Wind with the knob in its upper range, because the 300:1 gearbox already sets the speed.
- **Mains safety:** use a sealed external adapter only. No mains voltage enters the box.
