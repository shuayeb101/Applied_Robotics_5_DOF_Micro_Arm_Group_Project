// =====================================================================
// Mini geared claw for the CT6056 5-DOF micro arm (Task 10, concept C6)
//
// A scaled-down "Klaw"-style angular gripper sized for an SG90:
//   * the servo turns a 10-tooth pinion (layer 1)
//   * the pinion drives the RIGHT jaw's gear (layer 1)
//   * the right jaw's gear drives the LEFT jaw's gear (layer 2)
//   so the jaws always open and close as a mirrored pair, and the grip
//   centre stays on the servo shaft line, the same as the original
//   rack-and-pinion gripper.
//
// Style taken from the reference photo: curved pointed jaws, serrated
// inner edges, round lightening holes, "stacked plate" grooves, and screw
// pivots sandwiched between two plates.
//
// The bottom plate contains the original FrontPlate's footprint, servo
// window and screw holes, so the servo sits exactly as before.
// CHECK how FrontPlate attaches to ArmFront before printing.
//
// Units: mm, degrees. Every value marked MEASURE is an estimate.
//
// Usage (OpenSCAD 2021+):
//   part = "assembly" | "pinion" | "jaw_right" | "jaw_left"
//        | "bottom_plate" | "top_plate" | "plate_2d" | "pinion_2d"
//        | "check_l1_pinion_jawR" | "check_l1_pinion_jawL"
//        | "check_l1_jaws" | "check_l2_jaws" | "check_drop_plate"
//   theta = jaw opening angle (0 = closed, theta_max = fully open)
//   e.g.  openscad -D 'part="jaw_right"' -o jaw_right.stl mini_claw.scad
//   The check_* parts are 2D intersections that must come out EMPTY.
//
// AI-assisted (Claude). Check every dimension before printing (C12).
// =====================================================================

part  = "assembly";
theta = 0;              // jaw opening angle, deg, 0 = closed
theta_max = 30;         // design opening range

// ---------------- gears ----------------
m        = 0.9;   // module (matches the original GripperGear: Ø10.8 tips, 10 teeth)
pa       = 20;    // pressure angle
z_p      = 10;    // pinion teeth
z_j      = 15;    // jaw gear teeth (full-circle equivalent)
backlash = 0.15;  // total backlash per mesh, split between the two gears
add_l1   = 0.75;  // jaw gear addendum factor in layer 1 (shortened so the
                  // 15T tips don't dig into the 10T pinion below its base circle)
add_l2   = 1.00;  // jaw gear addendum factor in layer 2 (jaw-to-jaw mesh)
layer    = 3;     // thickness of each gear layer (same as the original pinion)

// ---------------- SG90 servo (MEASURE) ----------------
tab_t                 = 2.5;   // mounting tab thickness
tab_top_to_spline_top = 10.5;  // MEASURE: top of tab to top of output spline
spline_engage         = 2.0;   // depth of the pinion's spline recess
shaft_from_win_end    = 5.9;   // MEASURE: shaft centre to the near end of the window

// ---------------- plates ----------------
plate_t = 2.0;
play    = 0.3;    // vertical running clearance for the jaws

// ---------------- jaws ----------------
jaw_len      = 30;   // pivot to tip, along the jaw
tip_gap      = 0.4;  // gap between tips when theta = 0
inner_start  = 9;    // where the gripping edge starts (local y)
inner_off    = 2.2;  // gripping edge x offset from the closed centreline at its start
w_root       = 7.0;  // finger width at inner_start
w_tip        = 2.6;  // finger width near the tip
finger_drop  = 3.0;  // extra finger height below the gear stack (taller grip face)
drop_from_y  = 13;   // local y where the taller finger section starts
serr_pitch   = 2.0;  // serration pitch on the gripping edge
serr_depth   = 0.8;  // serration depth
tip_curl     = 30;   // angle of the curled end section, inward from the jaw axis (deg)
curl_len     = 9;    // length of the curled end section
grooves      = true; // "stacked plate" grooves around the finger
groove_d     = 0.5;  // groove depth
groove_h     = 0.6;  // groove height

// ---------------- pivots ----------------
screw_d    = 2.0;   // M2 pivot screws
pilot_d    = 1.6;   // self-tapping pilot hole in the sleeve
sleeve_r   = 2.0;   // axle sleeve radius (printed on the bottom plate)
hub_clear  = 0.25;  // radial clearance jaw hub <-> sleeve
pillar_r   = 3.0;   // shoulder the jaw sits on

$fn = 48;

// =====================================================================
// derived values
// =====================================================================
rp_p  = m*z_p/2;               // 4.5
rp_j  = m*z_j/2;               // 6.75
a     = rp_j;                  // pivots at x = +-a (jaw gears touch at x = 0)
c_pj  = rp_p + rp_j;           // pinion-to-jaw centre distance
h     = sqrt(c_pj*c_pj - a*a); // pivot y, pinion at the origin
pivR  = [ a, h];
pivL  = [-a, h];

z0 = tab_t + tab_top_to_spline_top - spline_engage;  // bottom of gear stack
z1 = z0 + layer;                                     // layer 1 | layer 2
z2 = z1 + layer;                                     // top of jaws
zt = z2 + play;                                      // underside of top plate

dir_R_to_pin = atan2(-h, -a);  // ~233 deg
dir_R_to_L   = 180;

function inv_rad(x) = tan(x) - x*PI/180;

// =====================================================================
// involute gear, 2D. One tooth centred on +x (angle 0) then rotated
// by `phase` degrees.
// =====================================================================
function flank_ang(r, rb, psi) =
    let(t = sqrt(max(0, (r/rb)*(r/rb) - 1)))
    (psi - (t - atan(t)*PI/180)) * 180/PI;

module gear2d(z, addf=1, phase=0) {
    rp  = m*z/2;
    rb  = rp*cos(pa);
    ra  = rp + addf*m;
    rf  = rp - 1.25*m;
    s   = PI*m/2 - backlash/2;        // tooth thickness at the pitch circle
    psi = s/(2*rp) + inv_rad(pa);      // half-tooth angle at the base circle, rad
    r0  = max(rb, rf);
    N   = 8;
    rs  = [for (i=[0:N]) r0 + (ra-r0)*i/N];
    pts = [for (k=[0:z-1]) let(c = k*360/z + phase) each concat(
              [[rf*cos(c - psi*180/PI), rf*sin(c - psi*180/PI)]],
              [for (r = rs)            [r*cos(c - flank_ang(r, rb, psi)), r*sin(c - flank_ang(r, rb, psi))]],
              [for (i=[N:-1:0]) let(r = rs[i]) [r*cos(c + flank_ang(r, rb, psi)), r*sin(c + flank_ang(r, rb, psi))]],
              [[rf*cos(c + psi*180/PI), rf*sin(c + psi*180/PI)]]
          )];
    polygon(pts);
}

module wedge2d(a0, a1, r=50) {
    polygon(concat([[0,0]], [for (i=[0:12]) let(t = a0 + (a1-a0)*i/12) [r*cos(t), r*sin(t)]]));
}

// =====================================================================
// jaw (right jaw, local frame: pivot at origin, jaw pointing +y)
// =====================================================================
x_tip  = tip_gap/2 - a;                    // tip x in local frame
P_tip  = [x_tip, jaw_len];                 // tip
cd     = [-sin(tip_curl), cos(tip_curl)];  // direction of the curled end, towards the tip
cn     = [ cos(tip_curl), sin(tip_curl)];  // its outward normal
P_bend = P_tip - curl_len*cd;              // where the straight edge turns into the curl
P_in0  = [-a + inner_off, inner_start];    // start of gripping edge
P_out0 = [P_in0[0] + w_root, inner_start];

function lerp(p, q, t) = p + (q-p)*t;
function width_at(y) = let(t = (y - inner_start)/(jaw_len - 4.5 - inner_start))
                       w_root + (w_tip - w_root)*t;

P_bend_o = P_bend + [width_at(P_bend[1]), 0];
P_tip_o  = P_tip + cn*(0.6*w_tip) - cd*3;

module finger_body2d() {
    hull() { circle(r = 4.6); translate(P_in0) circle(r=0.01); translate(P_out0) circle(r=0.01); }
    polygon([P_in0, P_bend, P_tip, P_tip_o, P_bend_o, P_out0]);
}

module serr_seg(p, q, end_margin) {
    L = norm(q - p);
    n = floor((L - end_margin) / serr_pitch);
    ang = atan2(q[1]-p[1], q[0]-p[0]);
    if (n >= 1) for (i = [1:n]) {
        c = lerp(p, q, (i*serr_pitch)/L);
        translate(c) rotate(ang) rotate(45) square(serr_depth*sqrt(2), center=true);
    }
}

module serrations2d() {
    serr_seg(P_in0, P_bend, 1);
    serr_seg(P_bend, P_tip, 2.5);   // the curled end is serrated too, but not the point
}

module holes2d() {
    for (y = [13, 17.5, 22]) if (y < P_bend[1] - 1.5) {
        t  = (y - P_in0[1])/(P_bend[1] - P_in0[1]);
        xi = P_in0[0] + (P_bend[0]-P_in0[0])*t;
        w  = width_at(y);
        r  = min(1.5, (w - 2.6)/2);
        if (r >= 0.6) translate([xi + w/2, y]) circle(r = r);
    }
}

module finger2d() {
    difference() { finger_body2d(); serrations2d(); holes2d(); }
}

module drop2d() {   // part of the finger that also extends below the gear stack
    intersection() { finger2d(); translate([-20, drop_from_y]) square([40, 40]); }
}

module hub_hole2d() { circle(r = sleeve_r + hub_clear); }

// local sector limits: contact direction +/- travel and two teeth of margin
tooth = 360/z_j;

module jaw_layer2d(lay, right=true) {
    // right jaw opens clockwise (-theta); its teeth that contact the
    // mating gear sweep from dir to dir + theta_max in the local frame.
    difference() {
        union() {
            finger2d();
            circle(r = rp_j - 1.25*m - 0.2);      // hub disc
            if (lay == 2) {
                intersection() {
                    gear2d(z_j, add_l2, right ? 180 : 0);
                    wedge2d(180 - 2*tooth, 180 + theta_max + 2*tooth);
                }
            }
            if (lay == 1 && right) {
                intersection() {
                    gear2d(z_j, add_l1, dir_R_to_pin);
                    wedge2d(dir_R_to_pin - 2*tooth, dir_R_to_pin + theta_max + 2*tooth);
                }
            }
        }
        hub_hole2d();
    }
}

// The left jaw is the mirror image of the right jaw, except for its gear
// teeth: with an odd tooth count the mirrored gear would put a tooth
// against a tooth, so the left gear is built with a gap facing the right jaw.
module jawL_layer2d(lay) {
    difference() {
        union() {
            mirror([1,0]) finger2d();
            circle(r = rp_j - 1.25*m - 0.2);
            if (lay == 2) {
                intersection() {
                    gear2d(z_j, add_l2, tooth/2);   // gap centred on 0 deg (towards the right jaw)
                    wedge2d(-theta_max - 2*tooth, 2*tooth);
                }
            }
        }
        hub_hole2d();
    }
}

module stack_grooves() {   // "stacked plate" grooves around the finger only
    if (grooves) {
        for (z = [z0 - finger_drop + 1.5 : 1.5 : z2 - 1.0]) {
            translate([0,0,z]) linear_extrude(groove_h)
                difference() {
                    intersection() { offset(r = 3) finger2d(); translate([-20, inner_start + 1]) square([40, 40]); }
                    offset(delta = -groove_d) finger2d();
                    circle(r = 6);   // keep the hub and gear area solid
                }
        }
    }
}

module jaw_right_3d() {
    difference() {
        union() {
            translate([0,0,z0]) linear_extrude(layer) jaw_layer2d(1, true);
            translate([0,0,z1]) linear_extrude(layer) jaw_layer2d(2, true);
            translate([0,0,z0 - finger_drop]) linear_extrude(finger_drop + 0.01) drop2d();
        }
        stack_grooves();
    }
}

module jaw_left_3d() {
    difference() {
        union() {
            translate([0,0,z0]) linear_extrude(layer) jawL_layer2d(1);
            translate([0,0,z1]) linear_extrude(layer) jawL_layer2d(2);
            translate([0,0,z0 - finger_drop]) linear_extrude(finger_drop + 0.01) mirror([1,0]) drop2d();
        }
        mirror([1,0,0]) stack_grooves();
    }
}

// =====================================================================
// pinion: our involute teeth + the ORIGINAL GripperGear's spline hub,
// so it fits the SG90 exactly as the supplied gear does.
// Phase: a gap faces the right jaw at theta = 0.
// =====================================================================
dir_pin_to_R = atan2(h, a);   // ~53 deg
pin_phase    = dir_pin_to_R + 180/z_p;

module pinion2d() { gear2d(z_p, 1, pin_phase); }

module pinion_3d() {
    union() {
        difference() {
            linear_extrude(layer) pinion2d();
            translate([0,0,-1]) cylinder(r = 2.95, h = layer + 2);
        }
        // original hub, flipped so the spline recess faces the servo
        intersection() {
            translate([0,0,layer]) mirror([0,0,1])
                import("../../00_baseline/3d_models/GripperGear.stl");
            translate([0,0,-1]) cylinder(r = 3.15, h = layer + 2);
        }
    }
}

// =====================================================================
// plates
// =====================================================================
// Original FrontPlate pattern, re-expressed with the shaft at the origin:
// plate 43 x 20, window 23.4 x 12.6, two Ø2.0 holes 27.5 apart.
win_x0 = -(23.4 - shaft_from_win_end);
win_x1 = shaft_from_win_end;
fp_x0  = win_x0 - 5.8;
fp_x1  = win_x1 + 13.8;
hole_x = [win_x0 - 2.05, win_x0 - 2.05 + 27.5];

module servo_body2d() {   // keep-out for the servo case and tabs (MEASURE)
    translate([win_x0 - 4.9, -6.0]) square([23.4 + 9.8, 12.0]);
}

module bottom_plate2d() {
    difference() {
        union() {
            translate([fp_x0, -10]) square([fp_x1 - fp_x0, 20]);
            hull() {
                translate([-12, -2]) square([24, 12]);
                translate(pivR) circle(r = pillar_r + 1.5);
                translate(pivL) circle(r = pillar_r + 1.5);
            }
        }
        translate([win_x0, -6.3]) square([23.4, 12.6]);
        for (x = hole_x) translate([x, 0]) circle(d = 2.0);
        // lightening holes, styled after the reference photo
        for (q = [[13, -4.5], [13, 4.5]]) translate(q) circle(r = 2.2);
        translate([17.3, 0]) circle(r = 1.2);
        translate([0, h + 2]) circle(r = 2.4);
    }
}

module bottom_plate_3d() {
    translate([0,0,-plate_t]) linear_extrude(plate_t) bottom_plate2d();
    for (p = [pivR, pivL]) translate(p) difference() {
        union() {
            cylinder(r = pillar_r, h = z0);            // shoulder under the jaw hub
            cylinder(r = sleeve_r, h = z2 + play);     // axle sleeve through the hub
        }
        translate([0,0,-plate_t - 1]) cylinder(d = pilot_d, h = 40);
    }
}

module top_plate2d() {
    difference() {
        hull() {
            translate(pivR) circle(r = 5);
            translate(pivL) circle(r = 5);
            circle(r = 3.5);
        }
        for (p = [pivR, pivL]) translate(p) circle(d = screw_d + 0.3);
        circle(r = 2.6);                                 // screwdriver access to the pinion screw
        translate([0, h - 1]) circle(r = 2.2);           // lightening hole
    }
}

module top_plate_3d() { translate([0,0,zt]) linear_extrude(plate_t) top_plate2d(); }

// =====================================================================
// posed parts (theta = jaw opening, pinion turns 1.5x the jaw angle)
// =====================================================================
ratio = z_j/z_p;
module posedR() { translate(pivR) rotate(-theta) children(); }
module posedL() { translate(pivL) rotate( theta) children(); }
module posedP() { rotate(ratio*theta) children(); }

module assembly() {
    color("dimgray") bottom_plate_3d();
    color("gray", 0.5) top_plate_3d();
    color("silver") translate([0,0,z0]) posedP() pinion_3d();
    color("#3a3a3a") posedR() jaw_right_3d();
    color("#3a3a3a") posedL() jaw_left_3d();
    color("royalblue", 0.25) translate([0,0,tab_t]) linear_extrude(tab_top_to_spline_top - 3) servo_body2d();
}

// =====================================================================
// output
// =====================================================================
if (part == "assembly")      assembly();
if (part == "pinion")        pinion_3d();
if (part == "jaw_right")     jaw_right_3d();
if (part == "jaw_left")      jaw_left_3d();
if (part == "bottom_plate")  bottom_plate_3d();
if (part == "top_plate")     translate([0,0,-zt]) top_plate_3d();
if (part == "plate_2d")      bottom_plate2d();
if (part == "pinion_2d")     pinion2d();
if (part == "finger_2d")     finger2d();   // right jaw finger outline, local frame (used by sim/grip_sim.py)

// 2D collision checks: each must render EMPTY at every theta
if (part == "check_l1_pinion_jawR") intersection() { posedP() pinion2d(); posedR() jaw_layer2d(1, true); }
if (part == "check_l1_pinion_jawL") intersection() { posedP() pinion2d(); posedL() jawL_layer2d(1); }
if (part == "check_l1_jaws")        intersection() { posedR() jaw_layer2d(1, true); posedL() jawL_layer2d(1); }
if (part == "check_l2_jaws")        intersection() { posedR() jaw_layer2d(2, true); posedL() jawL_layer2d(2); }
if (part == "check_drop_plate")     intersection() {
                                        union() { bottom_plate2d(); servo_body2d(); }
                                        union() { posedR() drop2d(); posedL() mirror([1,0]) drop2d(); }
                                    }
