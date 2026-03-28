"""
Inverted Pendulum Race Car — Interactive Simulation
====================================================
Phases
------
  READY    : pendulum rests horizontal.  Nothing moves.
  SWING-UP : SPACE pressed — reaction motor swings pendulum from horizontal
             to upright using energy-pumping control.  Cart locked.
  RACE     : pendulum is upright — LQR/PID stabilises it while you drive.

Keyboard controls
-----------------
  SPACE      : start race (READY → SWING-UP → RACE)
  LEFT / A   : drive cart left  (RACE phase only)
  RIGHT / D  : drive cart right (RACE phase only)
  P          : toggle LQR ↔ PID stabiliser
  R          : reset to READY
  Q / Esc    : quit

The physics runs at PHYSICS_HZ (500 Hz); the display updates at RENDER_FPS
(60 fps).  Simulated BNO055 IMU data is derived each frame from the physics
state, matching what the real sensor would output.
"""

import sys
import numpy as np

# ── Physical parameters ──────────────────────────────────────────────────────
# Cart mass includes chassis + wheels + battery + ESP32 + motor mounts (~2 kg).
# CART_FRIC is viscous rolling + drivetrain friction; at MAX_CART_FORCE it gives
# a natural terminal velocity of MAX_CART_FORCE / CART_FRIC ≈ 4 m/s so the cart
# never feels weightless.  At 2 m/s cart acceleration is ~(8 - 2*2*2)/2.3 ≈ 1.7 m/s²
# → pendulum sees an inertial tilt of arctan(1.7/9.81) ≈ 10°, which is physically
# significant and requires the reaction motor to correct.
CART_MASS   = 2.5          # M  (kg)  — heavier, realistic for car + payload
PEND_MASS   = 0.3          # m  (kg)
PEND_LEN    = 0.5          # L  (m)  full rod length
GRAVITY     = 9.81         # g  (m/s²)
CART_FRIC   = 2.0          # b_cart  (N·s/m)  — caps top speed at ~4 m/s
PEND_FRIC   = 0.01         # b_pend  (N·m·s/rad)

I_HINGE = (1.0 / 3.0) * PEND_MASS * PEND_LEN ** 2   # uniform rod about hinge

# ── Simulation parameters ────────────────────────────────────────────────────
PHYSICS_HZ      = 500          # physics integration rate
PHYSICS_DT      = 1.0 / PHYSICS_HZ
RENDER_FPS      = 60
STEPS_PER_FRAME = PHYSICS_HZ // RENDER_FPS   # 8 physics steps per display frame

MAX_TORQUE      = 5.0          # reaction motor saturation (N·m)
MAX_CART_FORCE  = 8.0          # max wheel-motor force (N) — realistic for small car
TRACK_HALF      = 4.0          # half-track length (m)  — slightly longer for the race

# ── Reaction motor model ─────────────────────────────────────────────────────
# The reaction motor sits at the pendulum hinge and applies torque directly.
# PWM from the ESP32 commands a target motor speed; the motor has a first-order
# speed response, and torque is proportional to commanded speed (back-EMF model).
#
#   omega_cmd  = clipped LQR/PID output, scaled to motor rad/s
#   omega_m    = actual motor speed (tracks omega_cmd with lag τ_m)
#   tau_pend   = Km * omega_m   (torque on pendulum hinge)
#
MOTOR_MAX_RPM  = 300.0                           # full-PWM no-load speed
MOTOR_MAX_RADS = MOTOR_MAX_RPM * 2.0 * np.pi / 60.0   # ≈ 314 rad/s
MOTOR_Km       = MAX_TORQUE / MOTOR_MAX_RADS     # torque constant  (N·m / rad·s⁻¹)
MOTOR_TAU      = 0.03                            # motor time constant (s)

# ── Swing-up controller ───────────────────────────────────────────────────────
# Energy-pumping swing-up (Åström style) for direct-torque actuator.
# The target is the total mechanical energy at the upright position.
#
E_UPRIGHT      = PEND_MASS * GRAVITY * PEND_LEN / 2.0   # PE at theta = 0
K_SWING        = 10.0                                     # energy-error gain
SWING_HANDOVER = np.radians(20.0)   # switch to LQR/PID when |theta| < this

# ── Simulation phases ─────────────────────────────────────────────────────────
PHASE_READY   = "READY"
PHASE_SWINGUP = "SWING-UP"
PHASE_RACE    = "RACE"

# ── Air resistance ───────────────────────────────────────────────────────────
# Linear drag on the pendulum rod/bob from motion through air.
# Integrated along the uniform rod; gives both a translational drag term
# (from cart velocity through still air) and a rotational drag term
# (from the rod sweeping through air):
#   tau_air = -PEND_AIR * [ xd*(L²/2)*cos(θ)  +  thd*(L³/3) ]
# The cart-induced term (xd part) is the "reaction force from cart acceleration"
# effect: when the cart brakes or accelerates, the pendulum sees an apparent wind
# that creates an extra restoring/destabilising torque.
PEND_AIR       = 0.06          # N·s/m² — air drag coefficient along rod

# ── Display constants ────────────────────────────────────────────────────────
SCREEN_W, SCREEN_H  = 1280, 680
TRACK_Y             = 470         # screen y of track surface
PPM                 = 120         # pixels per metre (world → screen)
PEND_PX             = 200         # visual pendulum length in pixels
CART_W, CART_H      = 90, 38

# Colours
C_BG      = (22,  26,  34)
C_TRACK   = (80,  88, 110)
C_GRID    = (35,  40,  52)
C_CART    = (70, 130, 200)
C_ROD     = (230, 190,  60)
C_BOB     = (240, 100,  55)
C_PIVOT   = (210, 210, 210)
C_HUD     = (210, 215, 225)
C_WARN    = (220,  60,  60)
C_OK      = (70,  200, 120)
C_PID_COL = (210, 155,  70)
C_WALL    = (180,  60,  60)


# ── Physics ──────────────────────────────────────────────────────────────────

def equations_of_motion(state, F_cart, tau_motor):
    """
    Full nonlinear EOM for cart + pendulum (uniform rod) with reaction motor.

    Parameters
    ----------
    state     : [x, x_dot, theta, theta_dot]
    F_cart    : force on cart from wheels (N)
    tau_motor : torque from reaction motor on pendulum hinge (N·m)

    Returns
    -------
    state_dot : [x_dot, x_ddot, theta_dot, theta_ddot]

    Physics notes
    -------------
    • Cart ↔ pendulum coupling is handled by the off-diagonal mass-matrix term
      M12 = (mL/2)cosθ.  When the cart accelerates (large f1), the pendulum
      sees  θ̈ += -M12*f1/det  — i.e. a backward inertial tilt.  This is the
      "reaction force from cart acceleration" on the pendulum.

    • Air drag on the uniform rod is integrated analytically:
        τ_air = -PEND_AIR * [ ẋ·(L²/2)·cosθ  +  θ̇·(L³/3) ]
      The first term couples cart speed to pendulum (apparent headwind/tailwind).
      The second term is rotational drag that supplements PEND_FRIC.
    """
    _, xd, th, thd = state
    M, m, L, g = CART_MASS, PEND_MASS, PEND_LEN, GRAVITY

    s, c = np.sin(th), np.cos(th)

    # Mass matrix
    M11 = M + m
    M12 = (m * L / 2.0) * c
    M22 = I_HINGE

    # Air drag torque on pendulum (integrated along uniform rod)
    tau_air = -PEND_AIR * (xd * (L**2 / 2.0) * c + thd * (L**3 / 3.0))

    # Generalised forces
    f1 = F_cart - CART_FRIC * xd + (m * L / 2.0) * thd**2 * s
    f2 = (m * g * L / 2.0) * s - PEND_FRIC * thd + tau_motor + tau_air

    det = M11 * M22 - M12 * M12

    x_ddot  = ( M22 * f1 - M12 * f2) / det
    th_ddot = (-M12 * f1 + M11 * f2) / det

    return np.array([xd, x_ddot, thd, th_ddot])


def rk4_step(state, F_cart, tau_motor, dt=PHYSICS_DT):
    """4th-order Runge-Kutta integration step."""
    k1 = equations_of_motion(state,               F_cart, tau_motor)
    k2 = equations_of_motion(state + 0.5*dt*k1,   F_cart, tau_motor)
    k3 = equations_of_motion(state + 0.5*dt*k2,   F_cart, tau_motor)
    k4 = equations_of_motion(state +    dt*k3,     F_cart, tau_motor)
    return state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)


def simulate_imu(state, state_prev, dt=PHYSICS_DT):
    """
    Generate simulated BNO055 sensor data from the physics state.

    Mirrors the fields that will be read from the real sensor on the ESP32.

    Returns
    -------
    dict with keys:
      euler_z   – pendulum tilt angle (deg)
      gyro_z    – angular velocity around hinge axis (deg/s)
      accel_x   – cart linear acceleration along track (m/s²)
      accel_z   – vertical acceleration seen at hinge (m/s²)
    """
    _, xd,  th,  thd  = state
    _, xdp, _, thdp   = state_prev

    x_ddot  = (xd  - xdp)  / dt if dt > 1e-9 else 0.0
    th_ddot = (thd - thdp) / dt if dt > 1e-9 else 0.0

    return {
        "euler_z" : float(np.degrees(th)),
        "gyro_z"  : float(np.degrees(thd)),
        "accel_x" : float(x_ddot),
        "accel_z" : float(GRAVITY + (PEND_MASS * PEND_LEN / 2.0) * th_ddot),
    }


# ── Swing-up control ─────────────────────────────────────────────────────────

def swing_up_torque(theta, theta_dot):
    """
    Energy-pumping swing-up controller.

    Computes the mechanical energy of the pendulum and pumps torque in the
    direction that closes the gap to the upright energy E_UPRIGHT.

    At zero angular velocity a small bias kick is applied so the pendulum
    always has a defined direction to swing toward.

    Returns
    -------
    tau : float — torque command (N·m), saturated to ±MAX_TORQUE
    """
    E_cur = (0.5 * I_HINGE * theta_dot ** 2
             + PEND_MASS * GRAVITY * (PEND_LEN / 2.0) * np.cos(theta))
    E_err = E_UPRIGHT - E_cur

    # Smooth sign via tanh so we don't chatter at theta_dot ≈ 0
    direction = np.tanh(theta_dot * 4.0)

    tau = K_SWING * E_err * direction
    return float(np.clip(tau, -MAX_TORQUE, MAX_TORQUE))


# ── Rendering ─────────────────────────────────────────────────────────────────

def _wx(world_x):
    """World x (m) → screen x (px)."""
    return int(SCREEN_W // 2 + world_x * PPM)


def _draw_background(screen):
    import pygame
    screen.fill(C_BG)
    # vertical grid lines
    for xi in np.arange(-TRACK_HALF - 1, TRACK_HALF + 2, 1.0):
        sx = _wx(xi)
        pygame.draw.line(screen, C_GRID, (sx, 0), (sx, SCREEN_H), 1)
    # track surface
    tl = _wx(-TRACK_HALF)
    tr = _wx( TRACK_HALF)
    pygame.draw.line(screen, C_TRACK, (tl, TRACK_Y), (tr, TRACK_Y), 4)
    # end walls
    for sx in (tl, tr):
        pygame.draw.line(screen, C_WALL, (sx, TRACK_Y - 50), (sx, TRACK_Y + 8), 4)
    # distance markers
    for xi in np.arange(-TRACK_HALF, TRACK_HALF + 0.5, 0.5):
        sx = _wx(xi)
        h  = 10 if xi == int(xi) else 5
        pygame.draw.line(screen, C_TRACK, (sx, TRACK_Y), (sx, TRACK_Y + h), 2)


def _draw_cart(screen, cart_x_world):
    import pygame
    cx = _wx(cart_x_world)
    # body
    rect = pygame.Rect(cx - CART_W // 2, TRACK_Y - CART_H, CART_W, CART_H)
    pygame.draw.rect(screen, C_CART, rect, border_radius=7)
    pygame.draw.rect(screen, (100, 160, 230), rect, 2, border_radius=7)
    # wheels
    for wx_off in [-28, 28]:
        pygame.draw.circle(screen, (45, 50, 65),  (cx + wx_off, TRACK_Y), 13)
        pygame.draw.circle(screen, (90, 98, 120), (cx + wx_off, TRACK_Y), 9)
        pygame.draw.circle(screen, (130,135,150), (cx + wx_off, TRACK_Y), 4)
    return (cx, TRACK_Y - CART_H)          # pivot point


def _draw_pendulum(screen, pivot, theta):
    import pygame
    px, py = pivot
    tip_x = int(px + PEND_PX * np.sin(theta))
    tip_y = int(py - PEND_PX * np.cos(theta))

    # shadow / glow
    pygame.draw.line(screen, (60, 50, 20), pivot, (tip_x + 2, tip_y + 2), 8)
    # rod
    pygame.draw.line(screen, C_ROD, pivot, (tip_x, tip_y), 6)
    # bob
    pygame.draw.circle(screen, (180,  70,  35), (tip_x, tip_y), 13)
    pygame.draw.circle(screen, C_BOB,            (tip_x, tip_y), 10)


def _draw_motor(screen, pivot, omega_motor, motor_angle_accum):
    """
    Draw the reaction motor at the pivot with a spinning rotor disc.

    omega_motor      : actual motor speed (rad/s)  — used to show spin speed visually
    motor_angle_accum: accumulated rotor angle (rad) — updated by caller each frame
    """
    import pygame
    px, py = pivot
    R_outer = 18    # outer disc radius (px)
    R_inner = 11    # inner ring
    R_dot   = 4     # off-centre marker dot radius

    # Stator (fixed housing) — always grey
    pygame.draw.circle(screen, (80,  85,  95), (px, py), R_outer)
    pygame.draw.circle(screen, (110, 115, 128), (px, py), R_outer, 2)

    # Rotor (spinning) — colour indicates direction
    rotor_col = C_OK if omega_motor >= 0 else C_WARN
    pygame.draw.circle(screen, rotor_col, (px, py), R_inner)

    # Off-centre dot to show rotation
    dot_x = int(px + (R_inner - R_dot - 1) * np.cos(motor_angle_accum))
    dot_y = int(py + (R_inner - R_dot - 1) * np.sin(motor_angle_accum))
    pygame.draw.circle(screen, (230, 230, 230), (dot_x, dot_y), R_dot)

    # Label
    # (no text here — shown in HUD)


def _draw_hud(screen, font_m, font_s, state, imu, omega_motor, pwm_pct, mode, fallen,
              phase=PHASE_RACE):
    """
    Parameters
    ----------
    omega_motor : actual reaction motor speed (rad/s)
    pwm_pct     : motor PWM command  (-100 % … +100 %)
    phase       : current simulation phase string
    """
    import pygame
    x, xd, th, thd = state
    deg     = np.degrees(th)
    rpm     = omega_motor * 60.0 / (2.0 * np.pi)
    a_col   = C_WARN if abs(deg) > 20 else C_HUD
    m_col   = C_OK   if mode == "LQR" else C_PID_COL
    rpm_col = C_OK   if rpm >= 0      else C_WARN

    phase_colours = {
        PHASE_READY:   (110, 115, 135),
        PHASE_SWINGUP: (230, 190,  60),
        PHASE_RACE:    C_OK,
    }
    ph_col = phase_colours.get(phase, C_HUD)

    # ── Left panel: pendulum state + IMU ────────────────────────────────────
    lines = [
        (f"Phase: {phase}",                           ph_col),
        (f"Mode : {mode}",                            m_col),
        ("─" * 22,                                    (60, 65, 80)),
        ("PENDULUM STATE",                             (130, 135, 150)),
        (f"Angle:  {deg:+7.2f} °",                    a_col),
        (f"ω_pend:{np.degrees(thd):+7.1f} °/s",      C_HUD),
        ("─" * 22,                                    (60, 65, 80)),
        ("CART STATE",                                 (130, 135, 150)),
        (f"x    : {x:+7.3f} m",                       C_HUD),
        (f"v    : {xd:+7.2f} m/s",                    C_HUD),
        ("─" * 22,                                    (60, 65, 80)),
        ("REACTION MOTOR (control output)",            (130, 135, 150)),
        (f"Speed : {rpm:+8.1f} RPM",                  rpm_col),
        (f"PWM   : {pwm_pct:+7.1f} %",               rpm_col),
        ("─" * 22,                                    (60, 65, 80)),
        ("IMU (BNO055 simulated)",                     (130, 135, 150)),
        (f"θ    : {imu['euler_z']:+7.2f} °",          C_HUD),
        (f"ω    : {imu['gyro_z']:+7.1f} °/s",        C_HUD),
        (f"a_x  : {imu['accel_x']:+7.2f} m/s²",      C_HUD),
    ]
    for i, (txt, col) in enumerate(lines):
        screen.blit(font_m.render(txt, True, col), (18, 18 + i * 26))

    # ── Controls legend (bottom-right) ───────────────────────────────────────
    ctrl = ["SPACE  : Start race",
            "← / → : Drive cart",
            "P      : Toggle LQR / PID",
            "R      : Reset  |  Q: Quit"]
    for i, txt in enumerate(ctrl):
        screen.blit(font_s.render(txt, True, (110, 115, 135)),
                    (SCREEN_W - 270, SCREEN_H - 115 + i * 26))

    # ── Phase status bar (top-centre) ─────────────────────────────────────────
    if phase != PHASE_RACE:
        hints = {
            PHASE_READY:   "Pendulum horizontal — press SPACE to start swing-up",
            PHASE_SWINGUP: "Swing-up in progress — cart locked until upright",
        }
        hint_txt = hints.get(phase, "")
        surf     = font_s.render(f"  {hint_txt}  ", True, ph_col)
        bg       = pygame.Surface((surf.get_width() + 4, surf.get_height() + 4))
        bg.fill((28, 32, 42))
        bx = SCREEN_W // 2 - bg.get_width() // 2
        screen.blit(bg,   (bx, 6))
        screen.blit(surf, (bx + 2, 8))

    # ── Motor RPM bar (right edge) ────────────────────────────────────────────
    bar_h   = 220
    bar_x   = SCREEN_W - 30
    bar_top = SCREEN_H // 2 - bar_h // 2
    pygame.draw.rect(screen, (40, 45, 58),
                     (bar_x - 10, bar_top, 20, bar_h), border_radius=4)
    frac   = float(np.clip(pwm_pct / 100.0, -1.0, 1.0))
    fill_h = int(abs(frac) * bar_h / 2)
    mid    = bar_top + bar_h // 2
    col    = C_OK if frac >= 0 else C_WARN
    if fill_h > 1:
        pygame.draw.rect(screen, col,
                         (bar_x - 8,
                          mid - fill_h if frac >= 0 else mid,
                          16, fill_h), border_radius=3)
    pygame.draw.line(screen, C_HUD, (bar_x - 12, mid), (bar_x + 12, mid), 1)
    # bar label
    screen.blit(font_s.render("PWM", True, (110,115,135)), (bar_x - 18, bar_top - 20))

    # ── Fallen banner ─────────────────────────────────────────────────────────
    if fallen:
        txt_surf = font_m.render("  PENDULUM FALLEN — Press R to reset  ", True, (255, 255, 255))
        bg       = pygame.Surface((txt_surf.get_width() + 20, txt_surf.get_height() + 10))
        bg.fill(C_WARN)
        bx = SCREEN_W // 2 - bg.get_width() // 2
        screen.blit(bg,       (bx, 18))
        screen.blit(txt_surf, (bx + 10, 23))


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    import pygame
    from lqr_design     import compute_lqr_gains
    from pid_controller import CascadePID

    pygame.init()
    screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
    pygame.display.set_caption("Inverted Pendulum Race Car — Simulation")
    clock  = pygame.time.Clock()
    font_m = pygame.font.SysFont("Consolas", 20)
    font_s = pygame.font.SysFont("Consolas", 17)

    K_lqr = compute_lqr_gains()
    pid   = CascadePID(max_output=MAX_TORQUE)

    def make_ready_state():
        # Cart starts at the left end of the track (race start line).
        # Pendulum horizontal (theta = π/2, pointing right), zero velocity.
        return np.array([-TRACK_HALF + 0.3, 0.0, np.pi / 2.0, 0.0])

    state             = make_ready_state()
    state_prev        = state.copy()
    phase             = PHASE_READY
    mode              = "LQR"          # stabiliser choice (LQR or PID)
    omega_motor       = 0.0            # actual reaction motor speed (rad/s)
    motor_angle_accum = 0.0            # accumulated rotor angle for visual

    running = True
    while running:

        # ── Events ──────────────────────────────────────────────────────────
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    running = False

                if event.key == pygame.K_r:
                    state             = make_ready_state()
                    state_prev        = state.copy()
                    phase             = PHASE_READY
                    omega_motor       = 0.0
                    motor_angle_accum = 0.0
                    pid.reset()

                if event.key == pygame.K_SPACE and phase == PHASE_READY:
                    # Give pendulum a small upward kick to seed the swing-up
                    state[3] = -0.5    # theta_dot < 0  →  moving toward upright
                    phase    = PHASE_SWINGUP
                    pid.reset()

                if event.key == pygame.K_p:
                    mode = "PID" if mode == "LQR" else "LQR"
                    pid.reset()

        # ── Cart force: only available once pendulum is upright ──────────────
        keys   = pygame.key.get_pressed()
        F_user = 0.0
        if phase == PHASE_RACE:
            if keys[pygame.K_LEFT]  or keys[pygame.K_a]: F_user = -MAX_CART_FORCE
            if keys[pygame.K_RIGHT] or keys[pygame.K_d]: F_user =  MAX_CART_FORCE

        # ── Physics sub-steps ────────────────────────────────────────────────
        for _ in range(STEPS_PER_FRAME):

            if phase == PHASE_READY:
                # Nothing moves — hold position
                tau_cmd = 0.0

            else:
                x   = state[0]
                th  = state[2]
                thd = state[3]

                # Soft track walls (active in RACE only)
                if phase == PHASE_RACE:
                    if x < -TRACK_HALF: F_user = max(F_user,  8.0)
                    if x >  TRACK_HALF: F_user = min(F_user, -8.0)

                if phase == PHASE_SWINGUP:
                    tau_cmd = swing_up_torque(th, thd)
                    # Hand over to stabiliser once close enough to upright
                    if abs(th) < SWING_HANDOVER:
                        phase = PHASE_RACE
                        pid.reset()

                else:  # PHASE_RACE
                    if mode == "LQR":
                        tau_cmd = float(-K_lqr @ state[2:4])
                    else:
                        tau_cmd = pid.compute(th, thd, PHYSICS_DT)

                tau_cmd = float(np.clip(tau_cmd, -MAX_TORQUE, MAX_TORQUE))

            # ── Reaction motor dynamics ──────────────────────────────────────
            omega_cmd    = tau_cmd / MOTOR_Km
            omega_motor += (omega_cmd - omega_motor) * PHYSICS_DT / MOTOR_TAU
            omega_motor  = float(np.clip(omega_motor, -MOTOR_MAX_RADS, MOTOR_MAX_RADS))
            tau_actual   = MOTOR_Km * omega_motor

            state_prev = state.copy()
            state      = rk4_step(state, F_user, tau_actual, PHYSICS_DT)

        # ── Derived display values ───────────────────────────────────────────
        imu     = simulate_imu(state, state_prev, PHYSICS_DT)
        fallen  = (phase == PHASE_RACE) and abs(state[2]) > np.radians(45)
        pwm_pct = (omega_motor / MOTOR_MAX_RADS) * 100.0

        motor_angle_accum += omega_motor * (STEPS_PER_FRAME * PHYSICS_DT)

        # ── Render ───────────────────────────────────────────────────────────
        _draw_background(screen)
        pivot = _draw_cart(screen, state[0])
        _draw_pendulum(screen, pivot, state[2])
        _draw_motor(screen, pivot, omega_motor, motor_angle_accum)
        _draw_hud(screen, font_m, font_s, state, imu,
                  omega_motor, pwm_pct, mode, fallen, phase)

        pygame.display.flip()
        clock.tick(RENDER_FPS)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
