# Phoenix Tuner X: A Beginner's Guide to Configuring an FRC Robot

For teams that have never done this before. Written against the 2026 Phoenix 6
documentation. Every number and blink code here comes from CTRE's official docs,
linked at the end.

---f

## What this is and why you need it

Your robot's motors and sensors are not wired individually back to the roboRIO.
They all share a single pair of wires called the **CAN bus**, and they talk to
the roboRIO over that shared pair. Because they share one wire pair, each device
needs its own address so the roboRIO can tell them apart.

**Phoenix Tuner X** is the app that lets you talk to those devices directly:
give them addresses, update their firmware, change their settings, see what they
think is wrong, and spin a motor without writing any code.

Devices it works with (all made by CTR Electronics, also called CTRE):

- **Talon FX** — the motor controller built into Falcon 500, Kraken X60, and Kraken X44 motors
- **Talon FXS** — motor controller for other motors
- **CANcoder** — absolute rotation sensor, used on swerve steering and arms
- **Pigeon 2.0** — gyro, tells the robot which way it's facing
- **CANivore** — USB-to-CAN adapter, lets your laptop talk to CAN devices directly
- **CANdi, CANrange, CANdle** — sensor and LED devices

If your robot uses REV motors (SPARK MAX / SPARK Flex) instead, Tuner X is not
your tool — you want **REV Hardware Client 2**. (The original REV Hardware
Client has reached end of life. RHC2 needs REV devices on 26.x firmware or
newer; devices still on 2025 firmware have to be updated through Recovery Mode
first.) Many robots have both CTRE and REV hardware and need both apps.

---

## Before you start

Do not open Tuner X until all of this is true. Most "Tuner isn't working"
problems are actually one of these.

**Mechanical and electrical**

- Every CAN device is wired into the bus: green to green, yellow to yellow, at
  every single connection point. Mixing up green and yellow is, per CTRE's own
  docs, a common failure point during bring-up.
- Red and black power leads are not reversed anywhere.
- Breakers are installed in the PDP/PDH where needed.
- Battery is charged. A weak battery produces symptoms that look like software
  problems.
- **Bus termination exists at both ends.** The bus needs a 120 Ω resistor at
  each end. The roboRIO, CTRE's PDP, and the CANivore each have one built in, so
  a normal roboRIO-to-PDP robot is already terminated. To verify: with the robot
  **powered off**, measure resistance between CANH and CANL. You should read
  about **60 Ω** (two 120 Ω resistors in parallel). If you read 120 Ω, only one
  end is terminated — check that both the roboRIO and PDP are actually in the
  circuit and that the PDP's termination jumper is in the right position.
  **If you're using a REV PDH instead of a CTRE PDP**, the jumper advice above
  doesn't apply — check REV's documentation for how the PDH handles termination.
  The 60 Ω measurement is still the test.

**Software on your laptop**

- WPILib for the current season is installed.
- **FRC Game Tools** is installed. This one package contains the FRC Driver
  Station and the roboRIO Imaging Tool — they aren't separate downloads. Game
  Tools is **Windows only**, and the Driver Station is a hard requirement for
  enabling motors later.
- The roboRIO has been imaged for the current season and set to your team number.

**Know before you go**

- Write down which motor is which *physically* before you start. "Front left
  drive," "front left steer," "elevator," and so on. You will need this in step 6
  and it is much harder to work out later.
- Budget real time. A first-time bring-up on a full swerve robot is a multi-day
  job, not an afternoon.

---

## Safety rules

These are not optional and they are not paperwork. A Kraken can break fingers
and tear mechanisms apart.

- Robot on blocks, wheels off the ground, before any motor is enabled.
- Everyone's hands and tools clear of the mechanism before enabling.
- One person on the Driver Station with a finger on the enable/disable, watching
  the mechanism — not watching the laptop screen.
- Set current limits *before* first movement. A mechanism that hits a hard stop
  at full current can wreck itself very quickly.
- Test at low output first. Start at 10% and work up.
- Arms, elevators, and anything with stored energy: know where it will fall
  before you enable it.

---

## Words you'll see and what they mean

- **CAN bus** — the shared two-wire network all these devices talk on. Green
  (CANL) and yellow (CANH).
- **CAN ID / device ID** — a device's address on the bus. Range is **0 to 62**.
  Every device ships with ID **0**, which is why a fresh robot has conflicts.
- **Vendordep** — short for "vendor dependency." The code library that lets your
  robot program talk to CTRE devices. Installed into your robot project in VS Code.
- **Firmware** — the software running inside the motor controller itself, separate
  from your robot code. Updated through Tuner X.
- **Diagnostic server** — a small program on the roboRIO that Tuner X talks to.
  Without it, Tuner X cannot see your devices. More on this in step 2.
- **Field upgrade** — CTRE's name for flashing new firmware onto a device.
- **Fault** — a status flag on a device saying something happened. A **live**
  fault is happening right now. A **sticky** fault happened at some point and
  stays flagged until you clear it, like a check-engine code.
- **Neutral mode** — what a motor does when told to stop. **Brake** resists
  motion; **Coast** freewheels.
- **Stator current** — current flowing in the motor windings. Proportional to
  **torque**. Limit this to protect mechanisms and reduce heat.
- **Supply current** — current drawn from the battery. Limit this to avoid
  brownouts and tripped breakers.
- **Soft limit** — a position boundary enforced in software. The motor refuses to
  drive past it.
- **Hard limit** — a physical limit switch.
- **FRC lock** — a safety state. Once a device has been connected to a roboRIO,
  it will not spin unless the FRC Driver Station is enabled.
- **Pro / licensing** — paid extra features. All devices work with the free
  Phoenix 6 API; a license *adds* capability rather than unlocking basic function.

---

## Step 1 — Install the software

**Phoenix Tuner X** installs from an app store:

- Windows 11 — Microsoft Store, or the Phoenix Offline Installer from CTRE's
  GitHub releases
- Android 9.0+ — Google Play Store
- macOS 14.0+ and iOS 15.0+ — Apple App Store

So Mac and iPad users can run Tuner X. But note the split: Tuner X runs on
macOS, while **FRC Game Tools (and therefore the Driver Station) is
Windows-only**. Since enabling a motor requires the Driver Station, a Mac-only
team can configure devices but cannot spin a motor. CANivore is also unsupported
on macOS. Plan on having at least one Windows laptop.

**Phoenix 6 vendordep** goes into your robot project:

1. Open your robot project in WPILib VS Code.
2. Click the WPILib icon in the left sidebar.
3. Under **Available Dependencies**, find **CTRE-Phoenix (v6)** and click
   **Install**.
4. To update it later, use the **To Latest** button on the same page.

Notes:

- If you have older v5 devices, install **CTRE-Phoenix (v5)** alongside it. Both
  can coexist. Teams using *only* v5 devices still need the v6 vendordep
  installed.
- The offline route is the Phoenix Offline Installer from GitHub, then adding the
  vendordep as an offline library in VS Code.
- Python teams: `python3 -m pip install phoenix6`.

**How you know it worked:** Tuner X opens, and your robot project builds without
errors about missing CTRE imports.

---

## Step 2 — Get Tuner X talking to the robot

This is the step that confuses everyone, because there are two separate things
that must be true: Tuner must be *pointed at* the roboRIO, and the **diagnostic
server** must be *running on* the roboRIO.

### Pointing Tuner at the robot

In the upper-left **flyout menu** (in 2026 Tuner X this is minimized by default
and shows as a column of icons) there's a dropdown/textbox with presets:

- **Driver Station** — takes the robot IP from the FRC Driver Station, if it's running
- **roboRIO USB** — uses `172.22.11.2`, the roboRIO's address over a USB cable
- **localhost** — for simulation, or a CANivore plugged into your laptop

You can also type the robot's IP by hand. Over the radio it's usually
`10.TE.AM.2` — team 1234 is `10.12.34.2`, team 254 is `10.2.54.2`.

**Easiest first-time setup:** USB cable from laptop to roboRIO, pick **roboRIO
USB**. No radio, no network configuration, nothing else to go wrong.

### Getting the diagnostic server running

Tuner X cannot see devices without it. Two ways:

**Option A — deploy a robot program.** If your robot code creates even one CTRE
device object, the diagnostic server starts automatically. This is enough:

```java
private TalonFX m_motor = new TalonFX(0);
```

The ID doesn't even have to be a real device. You'll see something like this in
the RioLog or Driver Station console:

```
[phoenix-diagnostics] Server 2026.x.x running on port: 1250
```

**Option B — temporary diagnostic server.** Useful on a freshly imaged roboRIO
with no code on it yet. Point Tuner at the roboRIO IP and click **Run Temporary
Diagnostic Server**. It runs until the next reboot, then it's gone.

Don't change the port. It's 1250 and FRC users should leave it alone.

**Bench alternative (Windows only):** a CANivore plugged into your laptop's USB
lets you configure devices with no roboRIO at all — great for pre-assigning IDs
to a pile of motors before the robot exists. Enable the **CANivore USB** toggle
and set the target to **localhost**. Note that CANivore is **not supported on
macOS**, so a Mac team can't use this route. Configuring a CANivore also needs a
2023 roboRIO image or newer.

**How you know it worked:** Tuner X shows a connection, and the server version
appears at the bottom of the Devices page. That version year must match your
Tuner X year — 2026 Tuner needs a 2026 server.

**If nothing appears:** give the roboRIO about 30 seconds to finish booting and
check again.

---

## Step 3 — Find your devices

The **Devices** page is what Tuner X opens to. Each device is a card. The card's
**color** is doing real work here:

- **Green** — device has the latest firmware. Good.
- **Yellow** — newer firmware is available. Not broken, but see step 4.
- **Red** — **duplicate ID.** Two or more devices of the same model are both
  answering to the same address. There's also a message in the middle of the card.
  This is normal on a brand-new robot, because everything ships as ID 0.
- **Purple** — unexpected or beta firmware version.
- **Blue** — Tuner couldn't retrieve the list of available firmware. Check your
  internet connection first.

Click a card (or **View more details…** in grid view) to open **Device Details**,
which shows the name, model, ID, firmware version, and serial number.

**The Blink button is your friend.** It rapidly flashes that device's LEDs so you
can see which physical motor you're looking at. This is how you untangle a pile
of identical motors.

**Count your devices.** If you have twelve CAN devices on the robot and Tuner
shows nine, stop here and go to the troubleshooting section. Do not proceed with
a device missing — you will waste hours.

---

## Step 4 — Update firmware

Firmware is the software inside the device. It has to match the Phoenix 6 API
year in your robot project: **26.x firmware goes with the 2026 API and 2026
Tuner X.** Mismatches produce confusing errors that look like everything else.

Tuner X downloads the latest firmware automatically in the background when you
launch it — usually under ten seconds on a normal connection. You don't hunt for
files.

To update everything at once:

1. On the **Devices** page, tick the checkbox on a device card, then click the
   **checkmark icon in the top right** — that selects all devices of the same
   model. (With nothing ticked, the checkmark selects every device.)
2. Open the field-upgrade dialog. It lists each device's name, model, ID, and
   current firmware.
3. Pick the firmware **year** in the top-left selector — 2026 for the 2026 season.
4. Click **Update to latest**. (There's also a **Custom** year option if you need
   to pin a specific version per device model, but you rarely want this.)

Watch out for:

- Select **Phoenix 6** firmware if you're using the Phoenix 6 API, and Phoenix 5
  firmware for the v5 API. One robot project can use both.
- The "X" cancel button does **not** stop the device currently being flashed. It
  finishes that one and skips the rest.
- Don't unplug or power off mid-flash.

**How you know it worked:** every card turns green and reports the new version.

---

## Step 5 — Licensing (skip if you didn't buy anything)

All supported devices can use the Phoenix 6 API for free. Licensing **adds**
Phoenix Pro features; it doesn't unlock basic operation. If your team hasn't
bought Pro or a Season Pass, skip this step entirely.

There are three kinds of license:

- **Single Device** — licenses exactly one device. Fine for a handful of devices
  or benchtop work.
- **Season Pass** — FRC teams only. Tied to your team number, covers 100 devices.
- **CANivore Bus** — installed on a CANivore; **every compatible device on that
  CANivore's bus gets Pro without individual activation.** For a team with a lot
  of devices this is usually the cheapest route, and it survives swapping a
  device for a spare.

How to activate:

- The license icon is at the bottom right of each device card. Click it to open
  the licensing dialog, choose a seat, and activate.
- You can batch-activate across selected devices from the Devices page.
- **Season Pass:** go to the **Profile** page, click the license, enter your team
  number in the box below the list, and click **Assign Team**.
- License activation only works in Tuner X, not the old Tuner v1.
- Devices Tuner X has seen at least once appear in **Device History**, so you can
  license a device that isn't currently connected.

Two warnings worth taking seriously:

- **License activation is permanent and irreversible.**
- **Attaching a team number to a Season Pass is permanent.** And the roboRIO must
  be configured for that same team number, or devices won't show as Pro licensed.

---

## Step 6 — Assign CAN IDs and names

Every device ships as ID 0, so on a new robot you have to give each one a unique
address. This is tedious and there's no shortcut, but it's also the single most
valuable half hour of the whole process.

On the **Device Details** page there are textboxes for **Name** and **ID**.
Type the new value and press **Set**.

- Valid IDs are **0 through 62**.
- Strictly, a conflict is two devices *of the same model* sharing an ID — that's
  what CTRE's docs describe and what Tuner flags. But give every device a
  distinct ID anyway. It costs nothing and removes a whole category of confusion.
- Name them descriptively: `FL_Drive`, `FL_Steer`, `FL_Encoder`, `Elevator`,
  `Intake`. Tuner shows these names everywhere, and future-you will be grateful.

**The reliable method when everything is ID 0:** connect one device to the bus at
a time. CTRE documents two ways to isolate:

- *Best:* wire CAN from the roboRIO to one device only, power up, set that
  device's ID, then move to the next.
- *Faster:* leave the wiring alone and pull breakers (and the PCM fuse, and the
  CAN pigtail from the PDP) so only one device is powered at a time. Restore one
  device, power up, set its ID, repeat.

Two habits that pay off:

- **Physically label each device with its ID** as you go. Masking tape and a
  Sharpie is fine — Sharpie comes off with alcohol.
- **Write the whole map down** somewhere permanent: device, location on robot,
  CAN ID, Tuner name. Keep it with the robot. At competition this document is
  how you diagnose a problem in five minutes instead of fifty.

**How you know it worked:** no red cards on the Devices page, and Blink on each
device lights up the motor you expect.

---

## Step 7 — Configure the devices

The **Configs** tab lets you view, change, back up, restore, and factory-default
a device's settings. In 2026 Tuner X the configs are organized as a nested menu.
To apply changes, press the **apply button (the download icon)** on the top
button bar. More options live behind the **3-dots icon**.

### Decide this first: Tuner or code?

You can set configs in Tuner X, or in your robot code at startup. **Setting them
in code is the better default**, because:

- If a motor dies mid-competition and you swap in a spare, code re-applies every
  setting automatically. Tuner settings die with the old device.
- The settings are version-controlled with the rest of your project.
- Nobody has to remember what was clicked in an app three weeks ago.

Use Tuner configs for experimenting and for things that genuinely live on the
device, like a CANcoder magnet offset. Then put the final values in code. Pick
one approach per setting and be consistent, or you'll spend an evening wondering
why a value keeps changing back.

### What to set

**Inverted direction.** Talon FX supports clockwise and counter-clockwise invert,
judged *looking at the face of the motor*. Get this right before tuning anything.

**Neutral mode.** Brake or coast. Typical choices: brake on drivetrain and
anything that must hold position, coast on intakes and flywheels. Not a
safety-critical setting, but it changes how the robot feels to drive.

**Current limits.** This one matters enough to get its own section below.

**Soft limits.** Position boundaries for arms, elevators, turrets — anything that
can crash into itself. Set these before the mechanism moves under power. Your
code can read whether a soft limit is active via the forward/reverse soft limit
faults.

**Hard limits.** Two gotchas here:

- **The Talon FX with a Kraken X60 does not support hardware limit switches.** If
  you need one, use a remote sensor (CANcoder, CANdi, or CANrange acting as a
  limit) or override the limit in the control request from code.
- **Limits default to "normally open."** That means the switch has to be
  explicitly *closed* (grounded) for the motor to be neutraled. A rookie team
  wiring a switch and expecting it to stop the motor when open will find it does
  nothing.

**Feedback sensor source.** Which sensor the motor uses for position — its own
internal one, or a remote CANcoder.

**CANcoder magnet offset.** See step 9.

**Pigeon 2 mount calibration.** If your Pigeon isn't mounted perfectly flat and
square, run the **Pigeon 2.0 Calibration** page in Tuner X. An uncalibrated
gyro on a tilted mount gives you a heading that drifts in ways that look like a
code bug.

---

## How to choose current limits

First, two things that change how you think about this:

- **Devices already ship with default current limits configured.** A fresh motor
  is not unlimited. You're adjusting limits, not adding them from nothing.
- **Limits set too low hurt the robot.** CTRE's own example shows an 80 A stator
  limit cutting peak acceleration from about 170 rot/s² to about 75 rot/s². A team
  that clamps everything down "to be safe" builds a sluggish robot and won't know
  why. The goal is the right limit, not the lowest one.

CTRE is explicit that there is no universal right answer — "the optimal limits
depend on how the motor is integrated into the system." But they publish a worked
example, and it's a sane starting point:

- Four Krakens on swerve **drive** — 120 A stator, 70 A supply
- Four Krakens on swerve **steering** — 60 A stator, no supply limit
- One Kraken on an **elevator** — 80 A stator, 30 A supply
- One Kraken on an **intake** — 20 A stator, no supply limit

Things to understand about these:

- **Stator limits torque.** Lower it to stop a mechanism from destroying itself
  or from slipping the wheels, and to reduce heat.
- **Supply limits battery draw.** Lower it to avoid brownouts and tripped
  breakers. Brownout protection kicks in at 6.3 V on a roboRIO 1 and 6.75 V by
  default on a roboRIO 2.
- **The supply limiter has two stages,** which is how breaker protection actually
  works. `SupplyCurrentLimit` is the hard ceiling that prevents brownouts. If the
  limiter has been actively limiting for `SupplyCurrentLowerTime`, it drops to the
  lower ceiling `SupplyCurrentLowerLimit` until current falls below that — this is
  what keeps a sustained draw from tripping a breaker. If you only set
  `SupplyCurrentLimit`, you get brownout protection but not breaker protection.
- They're related by duty cycle: supply current ≈ stator current × duty cycle. At
  50% output, an 80 A stator draw is only 40 A from the battery. Supply current
  never exceeds the stator limit and is usually well below it.
- **Each limit must be separately enabled** by its own enable config. Setting the
  number without enabling it does nothing. This trips up a lot of teams.
- Because a stator limit already caps supply current, you often don't need both.

**To find your drivetrain's real limit** (CTRE's documented procedure):

1. Put the robot on carpet, against a wall.
2. Plot velocity and stator current in Tuner X.
3. Slowly increase voltage output until velocity goes above zero and stator
   current drops — that's the wheels slipping.
4. Set your stator limit somewhere below the current you measured at slip.

---

## Step 8 — Test, and read what the device tells you

### Self Test Snapshot

Run this on every device before you try to move anything. It shows the device's
immediate state — and helpfully, it renders the device's **status LEDs as an
animated GIF**, so you can read the blink code of a motor buried deep inside the
robot without a flashlight and a mirror.

Four buttons: **Refresh**, **Clear Faults** (which also blinks the device),
**Copy Self Test** (copies everything to your clipboard, handy for support
requests), and **Share to Support** (opens your email client addressed to CTRE
support).

### Faults, and why sticky ones matter

- A **live fault** is happening right now. It clears itself when the underlying
  problem is fixed.
- A **sticky fault** records that something happened and stays flagged until you
  clear it — like a check-engine light.

**Clear sticky faults after you finish wiring or mechanical changes.** Otherwise
you spend an hour chasing a fault from Tuesday. From code it's
`clearStickyFaults()`; note that call blocks, so don't put it in a periodic loop.

### Plotting

Graph position, velocity, current, and temperature live. This is the tool for
tuning and for finding mechanical problems — a graph of a binding mechanism looks
obviously wrong in a way that numbers scrolling past do not.

### Actually moving a motor

1. Open the device's **Control** page.
2. Click the red **DISABLED** button to switch it to **ENABLED**.
3. Pick a control mode from the dropdown.
4. Set output with the slider or the text entry. **Start low.**

**The thing everyone hits:** if there's a **lock icon** next to the button, that
device is **FRC locked**, and the **FRC Driver Station must also be enabled**
before it will move. Any device that has been connected to a roboRIO becomes FRC
locked. That is a safety feature, not a bug. (It can be cleared by
factory-defaulting the device in Tuner X — but on an FRC robot you generally
want it left on.)

So the real sequence is: enable in Tuner, *and* enable in Driver Station, then
the motor moves. Two enables.

### Tuning PID gains

Be realistic: this is a skill, not a step. Get everything above working first,
drive the robot, and come back to tuning later. Tuner X's plotting plus control
is the right environment for it, and CTRE has dedicated documentation on basic
PID, Motion Magic, and manual tuning. Don't let a rookie bring-up stall here.

---

## Step 9 — Swerve drivetrains

If you're running swerve, Tuner X's **Swerve Project Generator** will build a
working project for you. Use it. Writing swerve from scratch as a first-year team
is not a good use of your build season.

**Requirements — all of these must be true:**

- 8 Talon FX or Talon FXS (4 drive, 4 steer)
- 4 CANcoders, *or* 4 CANdi, *or* 4 Talon FXS with PWM encoders
- **Exactly 1 Pigeon 2.0.** This one is required, not optional.
- All devices on the **same CAN bus**
- All devices visible in Tuner X
- Firmware matching the year (26.x for 2026)
- Current-year diagnostics running
- Tuner connected to the robot

A CANivore and Pro licensing are **recommended but not required** — they improve
control quality, not basic function.

Tuner checks most of this for you with a mechanism called **precheck**. Hit the
**Refresh** button in the top right to re-run it.

**Configuring each module** is four picks: encoder, steer motor, drive motor,
then encoder calibration. An **Encoder Calibration** button appears once the
devices are picked, and it walks you through aligning the module and setting the
CANcoder offset.

Two warnings from CTRE that will cost you a day if you miss them:

- **Align the modules so the bevel gear faces the vertical center of the robot.**
  Get this wrong and the drive verification tests fail.
- **The generator factory-defaults your devices** in order to run its drive and
  steer tests. Back up your configs first if they aren't applied from code. (This
  is the best argument for keeping configs in code.)

Also: if you reassign an encoder to a different module, redo the encoder
calibration. Skipping it produces bizarre module behavior.

**Zeroing a CANcoder outside the generator:** the zero point is the **Magnet
Offset** config, and Tuner X has a button that sets it and reports the applied
offset. That button needs 2024 diagnostics or newer.

### Validating the drivetrain

After configuring modules the generator runs **Verify Steer** and **Verify
Drive** tests and asks you to confirm what you see. This is where rookie teams
get stuck, because the questions assume you know what correct looks like.

**Before you start:** robot on blocks, wheels off the ground, and **battery
above 11 V**. Also turn each drive and steer motor by hand and confirm it moves
freely — a stalled motor whistles or screeches, and that tells you something is
binding mechanically.

**Verify Steer:** the modules should rotate **counter-clockwise** when you look
down at the module from above.

**Verify Drive:** applies about 10% output. With the robot up on blocks, "forward"
means the wheels on the robot's **right side rotate clockwise** when viewed from
the right, and the **left side rotates counter-clockwise**.

Answering the prompts:

- Wheels didn't move at all → you forgot to enable in the Driver Station.
- Wheels spinning erratically or stuttering → check your battery voltage.
- **Both** sides rotating the wrong way → just answer "No" at the prompt; the
  generator handles it.
- **One** side wrong → redo that module's CANcoder calibration.

2026 notes: the generated project includes a default "drive straight" autonomous
that creeps forward for five seconds. C++ teams — `TunerConstants.h` now has to
explicitly include the motor controllers and encoder used.

---

## Step 10 — Sanity check before you call it done

- Every device appears in Tuner and every card is green.
- No duplicate IDs, and your written device map matches reality.
- Every device has a descriptive name.
- Sticky faults cleared, then re-run Self Test — still clean.
- Current limits set and **enabled** on every motor.
- Soft limits set on anything that can crash into itself.
- Each mechanism moves the right direction at low output.
- CAN bus utilization is healthy (see below).
- Robot code deploys and drives.
- Your device map is printed and physically with the robot.

---

## CAN bus utilization

**Keep total utilization below 90%.** Above that you get unexplained behavior
that's very hard to debug.

Rough per-device cost on a standard CAN 2.0 bus with Phoenix 6 defaults:

- Talon FX — about 4.1%
- Talon FXS — about 4.2%
- Pigeon 2 — about 3.1%
- CANdi — about 2.5%
- CANrange — about 2.0%
- CANcoder — about 1.7%
- CANdle — about 0.4% idle, more when animating LEDs
- The diagnostic server itself adds a constant 0–5%

Add that up for a swerve robot: 8 Talon FX is already about 33%, plus 4 CANcoders
and a Pigeon. It adds up faster than you'd think.

**CAN FD** roughly halves those numbers — Talon FX drops to about 1.8%. Two
things to understand before you buy a CANivore expecting a free upgrade:

- CAN FD needs a **CANivore**, and the roboRIO's own bus stays CAN 2.0. So the
  CANivore gives you a *second, separate* bus. You move devices onto it; you
  aren't flipping a switch on the bus you already have.
- Every device on that bus has to support CAN FD. One non-FD device on the bus
  and you don't get FD speeds. The Devices page shows a CAN FD icon on devices
  that support it (which indicates capability, not that they're currently on an
  FD bus).

---

## Troubleshooting

### A device doesn't appear in Tuner

Work through it in this order:

1. Is the diagnostic server actually running? Check the server version at the
   bottom of the Devices page.
2. Has the roboRIO finished booting? It takes about 30 seconds.
3. Is the device powered? Check its LEDs — off means no power.
4. **Check the wiring.** CTRE's list, and it's a good one:
   - Tug-test every crimped wire individually. Bad crimps are the most common
     cause of intermittent CAN problems.
   - Confirm green-to-green and yellow-to-yellow at every connection.
   - Confirm red and black aren't flipped.
   - Confirm breakers are installed.
   - Check battery voltage.
   - Measure CANH-to-CANL with the robot off: expect about 60 Ω.
5. Isolate it. Wire the roboRIO to that one device only. If it still doesn't
   appear, the device or its connector is the problem, not the bus.

**Test for intermittent connections** by flicking, shaking, and jostling sections
of the CAN harness while watching for red LED blips. A blip while you wiggle a
section means a loose contact right there.

### Red cards / duplicate IDs

Expected on a new robot — everything ships as ID 0. Fix by isolating devices and
assigning IDs one at a time (step 6).

### Motor won't spin from Tuner

Almost always the FRC lock. The Driver Station must be enabled *in addition to*
Tuner's enable button. Look for the lock icon.

### Reading TalonFX LEDs

There are two LEDs in the middle of the motor. A healthy, powered, uncontrolled
Talon FX **blinks orange** — so does a Pigeon 2.

| What you see | What it means |
|---|---|
| Off | No power. Check 12 V on the red/black leads. |
| Blinking alternating **red** | No valid CAN signal. Check green/yellow connections and that the roboRIO is on. |
| Blinking alternating **orange** | CAN is fine, but Phoenix isn't running on the roboRIO. Deploy a program that uses Phoenix, or start the diagnostic server. |
| Blinking **simultaneous orange** | CAN good, Phoenix running, device disabled. This is the healthy idle state. |
| Both solid orange | Enabled, neutral output. |
| Blinking simultaneous **green** | Driving forward. Blink rate tracks output. |
| Blinking simultaneous **red** | Driving in reverse. Blink rate tracks output. |
| Offset alternating red/off | Hit a limit — hard or soft. The direction of the offset tells you which one. |
| Offset orange/off | **Thermal cutoff.** Let it cool. Consider a stator current limit to reduce heat. |
| Alternating red/green | You sent a Pro-only command to an unlicensed device. |
| Alternating red/orange | **Damaged hardware.** Confirm with Self Test, then contact CTRE. |
| One LED alternating green/orange | Stuck in bootloader. Field-upgrade it in Tuner X. |

If the pattern doesn't exactly match any row, the device is probably alternating
between two codes — most often between good and bad CAN.

### Reading CANcoder LEDs

The CANcoder packs two messages into one LED, and the trick is **brightness**:

- **Bright** = CAN bus is healthy, and the color is telling you about magnet placement.
- **Dim** = CAN was never detected since boot, and the color is telling you about magnet placement.
- **Slow bright red** = no valid CAN.

| What you see | What it means |
|---|---|
| Off | No power. |
| Slow bright red | No valid CAN. Check green/yellow wiring. |
| Rapid **bright green** | CAN healthy, magnet placement good. This is what you want. |
| Rapid bright orange | CAN healthy, magnet in range but accuracy slightly reduced. Adjust distance. |
| Rapid bright red | CAN healthy, **magnet out of range.** Fix the magnet's position and alignment. |
| Rapid **dim** green/orange/red | Same magnet meanings, but CAN was never detected since boot. Fix the CAN wiring too. |
| Alternating red/orange | Damaged hardware. |
| Alternating orange/green | In bootloader. Field-upgrade it. |

Magnet strength bands: good is roughly 45–75 mT, reduced accuracy 25–45 or
75–135 mT, out of range below 25 or above 135 mT. **Wait 8 seconds after boot**
for the LED to start reporting magnet status.

### Firmware or version mismatch errors

Check that all three agree: device firmware year, vendordep year in your robot
project, and Tuner X year. For 2026 that's 26.x firmware, the 2026 vendordep,
and 2026 Tuner with a 2026 server version.

### Device shows as not licensed when it should be

- Latest diagnostic server running? Check the version at the bottom of the page.
- Latest vendordep and latest firmware?
- **Season Pass users: is the roboRIO configured with the correct team number?**
  A wrong team number on the robot makes licensed devices appear unlicensed.

---

## Where to get help

- **Phoenix 6 documentation** — https://v6.docs.ctr-electronics.com
- **CAN bus troubleshooting** —
  https://v6.docs.ctr-electronics.com/en/stable/docs/troubleshooting/canbus-troubleshooting.html
- **Tuner X section** — https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/index.html
- **Current limits** —
  https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
- **Swerve generator** —
  https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
- **Pigeon 2 calibration** —
  https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/pigeon-cal.html
- **CANivore setup** —
  https://v6.docs.ctr-electronics.com/en/stable/docs/canivore/canivore-setup.html
- **CTRE support** — support@ctr-electronics.com. Use **Copy Self Test** first
  and paste the output into your message; it's the single most useful thing you
  can include.
- **Chief Delphi** — https://www.chiefdelphi.com — the FRC community forum. Search
  your error message first; someone has almost certainly hit it.
- **Other teams.** At an event, ask. Nearly every veteran team will send someone
  over to help a rookie with a CAN problem. This is normal and expected.

---

## Quick checklist

Once you've read the above, this is the short version for next time.

**Prep**

- [ ] CAN wired, green-to-green and yellow-to-yellow
- [ ] Termination verified (~60 Ω, robot off)
- [ ] Battery charged, breakers in
- [ ] roboRIO imaged, team number set
- [ ] WPILib + Driver Station installed
- [ ] Robot on blocks

**Software**

- [ ] Tuner X installed
- [ ] Phoenix 6 vendordep installed in robot project

**Connect**

- [ ] Tuner pointed at robot (roboRIO USB is easiest)
- [ ] Diagnostic server running; server year matches Tuner year

**Devices**

- [ ] All devices appear; count matches the robot
- [ ] Blink-test each one against its physical location
- [ ] Firmware updated; all cards green
- [ ] Licenses activated, if you have any

**Configure**

- [ ] Unique CAN ID per device (0–62), physically labeled
- [ ] Descriptive names set
- [ ] Device map written down and kept with the robot
- [ ] Invert direction and neutral mode set
- [ ] Current limits set **and enabled**
- [ ] Soft limits set on anything that can crash
- [ ] Pigeon 2 mount calibration run, if it isn't mounted flat

**Verify**

- [ ] Self Test clean on every device
- [ ] Sticky faults cleared, then re-tested
- [ ] Each mechanism moves the right way at low output
- [ ] Swerve only: steer and drive verification passed, battery above 11 V
- [ ] CAN utilization under 90%
- [ ] Robot code deploys and drives
