const canvas = document.getElementById('simCanvas');
const ctx = canvas.getContext('2d');

// UI Elements
const ropeStrInput = document.getElementById('ropeStr');
const linkStrInput = document.getElementById('linkStr');
const stiffInput = document.getElementById('stiffness');
const weightInput = document.getElementById('weight');

// Label Updater
[ropeStrInput, linkStrInput, stiffInput, weightInput].forEach(el => {
    el.oninput = () => {
        const label = document.getElementById(el.id + 'Val') || document.getElementById(el.id.replace('Str', 'Val').replace('ness', 'Val'));
        if (label) label.innerText = el.value;
    };
});

// Constants
const G = 9.80665;
const DROP_HEIGHT_M = 22.86; // 75ft
const FLOOR_Y = DROP_HEIGHT_M + 5;
const SCALE = 25;
const DT = 0.008;
const M_TOP = parseFloat(weightInput.value) * 0.25; // 50lb static top mass

// Simulation State
let state; // [x1, y1, vx1, vy1, x2, y2, vx2, vy2]
let maxRope, maxLink, isRunning, isBrokenRope, isBrokenLink;
let shakeAmount = 0;

// Verlet Rope Points
const ROPE_SEGMENTS = 25;
const SEG_LEN = DROP_HEIGHT_M / ROPE_SEGMENTS;
let ropePoints = Array.from({ length: ROPE_SEGMENTS + 1 }, () => ({ x: 0, y: 0, oldX: 0, oldY: 0 }));

async function resetSim() {
    // Calculate center once
    const centerX_Pixels = canvas.width / 2;
    const centerX_Meters = centerX_Pixels / SCALE;

    // 1. Reset Physics State (Meters)
    // [x1, y1, vx1, vy1, x2, y2, vx2, vy2]
    state = [centerX_Meters + 0.01, 1, 0, 0, centerX_Meters + 0.001, 1.5, 0, 0];

    maxRope = 0;
    maxLink = 0;
    isRunning = true;
    isBrokenRope = false;
    isBrokenLink = false;
    shakeAmount = 0;

    // 2. Reset Rope Points (Pixels)
    ropePoints.forEach((p, i) => {
        p.x = p.oldX = centerX_Pixels;
        p.y = p.oldY = (i * SEG_LEN) * SCALE;
    });

    document.getElementById('status').innerText = "Falling...";
    document.getElementById('status').style.color = "#00ffcc";
}

function getAccelerations(s) {
    const [x1, y1, vx1, vy1, x2, y2, vx2, vy2] = s;
    const anchorX = (canvas.width / 2) / SCALE;
    const anchorY = 0;

    const rLimit = parseFloat(ropeStrInput.value);
    const lLimit = parseFloat(linkStrInput.value);
    const kRope = parseFloat(stiffInput.value);
    const mBot = parseFloat(weightInput.value) * 0.75;
    const kLink = 150000;

    // 1. Rope Vector Physics (2D)
    const dx1 = x1 - anchorX;
    const dy1 = y1 - anchorY;
    const dist1 = Math.sqrt(dx1 * dx1 + dy1 * dy1);

    let fRopeX = 0, fRopeY = 0;
    if (dist1 > DROP_HEIGHT_M && !isBrokenRope) {
        const stretch = dist1 - DROP_HEIGHT_M;
        // Radial velocity damping
        const vRel = (dx1 * vx1 + dy1 * vy1) / dist1;
        const tMag = kRope * stretch + (400 * vRel);

        if (tMag > 0) {
            fRopeX = -(tMag * (dx1 / dist1));
            fRopeY = -(tMag * (dy1 / dist1));
            if (tMag > rLimit) isBrokenRope = true;
            if (tMag > maxRope) maxRope = tMag;
            // Trigger camera shake on high impact
            //if (tMag > 5000) shakeAmount = Math.min(shakeAmount + (tMag / 2000), 15);
        }
    }

    // 2. Link Vector Physics (2D)
    const dxL = x2 - x1;
    const dyL = y2 - y1;
    const distL = Math.sqrt(dxL * dxL + dyL * dyL) || 0.0001;
    let stretchL = distL - 0.5;
    stretchL = Math.max(-0.05, Math.min(stretchL, 0.05)); const vRelL = (dxL * (vx2 - vx1) + dyL * (vy2 - vy1)) / distL;
    let tLinkMag = kLink * stretchL + (1000 * vRelL);
    const MAX_LINK_FORCE = 50000;
    tLinkMag = Math.max(-MAX_LINK_FORCE, Math.min(tLinkMag, MAX_LINK_FORCE));
    if (Math.abs(tLinkMag) > maxLink) maxLink = Math.abs(tLinkMag);
    if (Math.abs(tLinkMag) > lLimit) isBrokenLink = true;

    const fLinkX = isBrokenLink ? 0 : -(tLinkMag * (dxL / distL));
    const fLinkY = isBrokenLink ? 0 : -(tLinkMag * (dyL / distL));

    // 3. Environment (Drag & Floor)
    const dragK = 0.8;
    let fFloor1 = 0, fFloor2 = 0;
    if (y1 > FLOOR_Y) fFloor1 = 200000 * (y1 - FLOOR_Y) + 5000 * vy1;
    if (y2 > FLOOR_Y) fFloor2 = 200000 * (y2 - FLOOR_Y) + 5000 * vy2;

    return [
        vx1, vy1,
        (fRopeX - fLinkX - (dragK * vx1)) / M_TOP,
        (M_TOP * G + fRopeY - fLinkY - fFloor1 - (dragK * vy1)) / M_TOP,
        vx2, vy2,
        (fLinkX - (dragK * vx2)) / mBot,
        (mBot * G + fLinkY - fFloor2 - (dragK * vy2)) / mBot
    ];
}

function rk4Step() {
    const k1 = getAccelerations(state);
    const k2 = getAccelerations(state.map((v, i) => v + k1[i] * DT / 2));
    const k3 = getAccelerations(state.map((v, i) => v + k2[i] * DT / 2));
    const k4 = getAccelerations(state.map((v, i) => v + k3[i] * DT));
    state = state.map((v, i) => v + (DT / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]));
}

function updateVerletRope() {
    const centerX = (canvas.width / 2);
    // Move Points
    for (let i = 1; i < ropePoints.length; i++) {
        let p = ropePoints[i];
        let vx = (p.x - p.oldX) * 0.96;
        let vy = (p.y - p.oldY) * 0.96;
        p.oldX = p.x;
        p.oldY = p.y;
        p.x += vx;
        p.y += vy + (G * DT * DT * 60);
    }
    // Constraints
    for (let it = 0; it < 15; it++) {
        ropePoints[0].x = centerX; ropePoints[0].y = 0;
        if (!isBrokenRope) {
            let last = ropePoints[ropePoints.length - 1];
            last.x = state[0] * SCALE; last.y = state[1] * SCALE;
        }
        for (let i = 0; i < ropePoints.length - 1; i++) {
            let p1 = ropePoints[i], p2 = ropePoints[i + 1];
            let dx = p2.x - p1.x, dy = p2.y - p1.y;
            let d = Math.sqrt(dx * dx + dy * dy);
            let diff = (d - (SEG_LEN * SCALE)) / d;
            p1.x += dx * 0.5 * diff; p1.y += dy * 0.5 * diff;
            p2.x -= dx * 0.5 * diff; p2.y -= dy * 0.5 * diff;
        }
    }
}

async function draw() {
    canvas.width = canvas.offsetWidth;
    canvas.height = canvas.offsetHeight;
    const centerX = canvas.width / 2;

    if (isRunning) {
        rk4Step();
        updateVerletRope();

        document.getElementById('maxRope').innerText = `${Math.round(maxRope)} N`;
        document.getElementById('maxLink').innerText = `${Math.round(maxLink)} N`;
        document.getElementById('curVel').innerText = `${state[3].toFixed(1)} m/s`;

        if (Math.random() > 0.98) {
            state[2] += (Math.random() - 0.5) * 0.1; // Tiny kick to top mass velocity
        }

        if (isBrokenRope || isBrokenLink) {
            document.getElementById('status').innerText = "FAILED";
            document.getElementById('status').style.color = "#ff4d4d";
        }
    }

    // Camera Shake Apply
    ctx.save();
    /*if (shakeAmount > 0.1) {
        ctx.translate((Math.random() - 0.5) * shakeAmount, (Math.random() - 0.5) * shakeAmount);
        shakeAmount *= 0.9;
    }*/

    // Draw Floor
    ctx.strokeStyle = "#444";
    ctx.beginPath(); ctx.moveTo(0, FLOOR_Y * SCALE); ctx.lineTo(canvas.width, FLOOR_Y * SCALE); ctx.stroke();

    // Draw Rope
    if (!isBrokenRope) {
        ctx.beginPath();
        ctx.strokeStyle = (Math.sqrt(Math.pow(state[0] - (centerX / SCALE), 2) + Math.pow(state[1], 2)) > DROP_HEIGHT_M) ? "#ffcc00" : "#888";
        ctx.lineWidth = 3;
        ctx.moveTo(ropePoints[0].x, ropePoints[0].y);
        ropePoints.forEach(p => ctx.lineTo(p.x, p.y));
        ctx.stroke();
    }

    // Draw Top Mass
    ctx.fillStyle = "#eee";
    ctx.beginPath(); ctx.arc(state[0] * SCALE, state[1] * SCALE, 10, 0, Math.PI * 2); ctx.fill();

    // Draw Link
    if (!isBrokenLink) {
        ctx.strokeStyle = "#666"; ctx.lineWidth = 8;
        ctx.beginPath(); ctx.moveTo(state[0] * SCALE, state[1] * SCALE); ctx.lineTo(state[4] * SCALE, state[5] * SCALE); ctx.stroke();
    }

    // Draw Bottom Mass
    ctx.fillStyle = "#aaa";
    ctx.beginPath(); ctx.arc(state[4] * SCALE, state[5] * SCALE, 18, 0, Math.PI * 2); ctx.fill();

    ctx.restore();
    requestAnimationFrame(draw);
}
resetSim();

draw();

setTimeout(() => {
    resetSim();
}, 500); // 2000 milliseconds = 2 seconds


