// script.js
(() => {
    'use strict';

    // --- PHASE 1: Custom 2D Physics Engine ---
    // Step 1.1: Core 2D Math Library (Vec2)

    const Vec2 = {
        create: (x = 0, y = 0) => ({ x, y }),

        clone: (v) => ({ x: v.x, y: v.y }),

        add: (v1, v2) => ({ x: v1.x + v2.x, y: v1.y + v2.y }),
        sub: (v1, v2) => ({ x: v1.x - v2.x, y: v1.y - v2.y }),

        scale: (v, s) => ({ x: v.x * s, y: v.y * s }),
        invScale: (v, s) => ({ x: v.x / s, y: v.y / s }), // Inverse scale (division)

        dot: (v1, v2) => v1.x * v2.x + v1.y * v2.y,

        // 2D cross product returns a scalar: (v1.x * v2.y) - (v1.y * v2.x)
        // Useful for determining winding order or if a point is to the left/right of a vector
        crossScalar: (v1, v2) => (v1.x * v2.y) - (v1.y * v2.x),

        // Cross product of a vector and a scalar (for 2D angular dynamics: r x F_scalar)
        // Typically results in a vector perpendicular to r, scaled by F_scalar
        // Or for getting a perpendicular vector scaled: v x s = (-v.y * s, v.x * s)
        // For now, let's define a simple perpendicular vector function first.

        magnitudeSq: (v) => v.x * v.x + v.y * v.y,
        magnitude: (v) => Math.sqrt(v.x * v.x + v.y * v.y),

        normalize: (v) => {
            const mag = Vec2.magnitude(v);
            if (mag === 0) return Vec2.create(0, 0); // Or throw error, or return as is
            return Vec2.invScale(v, mag);
        },

        distanceSq: (v1, v2) => {
            const dx = v1.x - v2.x;
            const dy = v1.y - v2.y;
            return dx * dx + dy * dy;
        },
        distance: (v1, v2) => Math.sqrt(Vec2.distanceSq(v1, v2)),

        // Rotate a vector by an angle in radians around the origin
        rotate: (v, angleRad) => {
            const cosA = Math.cos(angleRad);
            const sinA = Math.sin(angleRad);
            return Vec2.create(
                v.x * cosA - v.y * sinA,
                v.x * sinA + v.y * cosA
            );
        },

        // Get a perpendicular vector (rotated 90 degrees counter-clockwise)
        perpendicular: (v) => Vec2.create(-v.y, v.x),

        // Linear interpolation
        lerp: (v1, v2, t) => Vec2.add(v1, Vec2.scale(Vec2.sub(v2, v1), t)),

        // Negate a vector
        negate: (v) => Vec2.create(-v.x, -v.y),

        // Project vector v1 onto vector v2
        project: (v1, v2) => {
            const magSqV2 = Vec2.magnitudeSq(v2);
            if (magSqV2 === 0) return Vec2.create(0,0);
            const dotProduct = Vec2.dot(v1, v2);
            return Vec2.scale(v2, dotProduct / magSqV2);
        }
    };
    console.log('[NEW_PDS_INIT] Vec2 math library defined.');

    let nextBodyId = 0; // Simple ID generator

    function createRigidBody(config) {
        const body = {
            id: nextBodyId++,

            // Kinematic properties
            position: config.position ? Vec2.clone(config.position) : Vec2.create(0, 0),
            angle: config.angle || 0, // Radians
            linearVelocity: config.linearVelocity ? Vec2.clone(config.linearVelocity) : Vec2.create(0, 0),
            angularVelocity: config.angularVelocity || 0, // Radians per second

            // Dynamic properties (accumulators, reset each step)
            force: Vec2.create(0, 0),
            torque: 0,

            // Mass properties
            mass: config.mass || 1, // Default to 1kg if not specified
            invMass: 0, // Will be set based on mass and isStatic
            inertia: config.inertia || 1, // Default to 1 kg*m^2 if not specified
            invInertia: 0, // Will be set based on inertia and isStatic

            // Shape / Geometry
            shapeType: config.shapeType || 'POLYGON', // 'POLYGON', 'CIRCLE'
            vertices: [], // Local-space vertices for polygons
            transformedVertices: [], // World-space vertices, updated each frame
            radius: config.radius || 0, // For circles

            // Physical Properties
            restitution: config.restitution || 0.2, // Bounciness
            staticFriction: config.staticFriction || 0.5,
            dynamicFriction: config.dynamicFriction || 0.3,

            // Material (placeholder for now)
            materialId: config.materialId || 'default_material',

            isStatic: config.isStatic || false,
            aabb: { min: Vec2.create(), max: Vec2.create() }, // World-space AABB
        };

        if (body.isStatic) {
            body.invMass = 0;
            body.invInertia = 0;
            body.mass = Infinity; // Conceptually
            body.inertia = Infinity; // Conceptually
        } else {
            if (body.mass <= 0) throw new Error("Dynamic body mass must be > 0");
            body.invMass = 1 / body.mass;
            if (body.inertia <= 0) throw new Error("Dynamic body inertia must be > 0");
            body.invInertia = 1 / body.inertia;
        }

        if (body.shapeType === 'POLYGON') {
            if (!config.vertices || config.vertices.length < 3) {
                throw new Error('Polygon shape requires at least 3 vertices.');
            }
            // Expect vertices to be relative to the body's geometric center (or desired CoM)
            body.vertices = config.vertices.map(v => Vec2.clone(v));
        } else if (body.shapeType === 'CIRCLE') {
            if (body.radius <= 0) {
                throw new Error('Circle shape requires a positive radius.');
            }
            // For circles, vertices can be generated for rendering/collision if needed,
            // but the primary geometric property is radius.
        } else {
            throw new Error(`Unsupported shapeType: ${body.shapeType}`);
        }

        // Initial update of transformed vertices
        updateTransformedVertices(body); // This now also updates AABB
        // updateAABB(body); // Call after updateTransformedVertices - No longer needed here as updateTransformedVertices calls it.

        console.log(`[RigidBody] Created ${body.isStatic ? 'static' : 'dynamic'} ${body.shapeType} body ID ${body.id}. AABB: min(${body.aabb.min.x.toFixed(1)},${body.aabb.min.y.toFixed(1)}), max(${body.aabb.max.x.toFixed(1)},${body.aabb.max.y.toFixed(1)})`);
        return body;
    }

    // Function to update world-space vertices from local-space vertices, position, and angle
    function updateTransformedVertices(body) {
        if (body.shapeType === 'POLYGON') {
            body.transformedVertices = body.vertices.map(vertex =>
                Vec2.add(body.position, Vec2.rotate(vertex, body.angle))
            );
        }
        // After vertices are transformed, update the AABB
        updateAABB(body);
    }

// New function to calculate/update AABB for a body
function updateAABB(body) {
    if (body.shapeType === 'POLYGON') {
        if (!body.transformedVertices || body.transformedVertices.length === 0) {
            if (body.vertices.length > 0 && body.transformedVertices.length === 0) {
                 body.transformedVertices = body.vertices.map(vertex =>
                    Vec2.add(body.position, Vec2.rotate(vertex, body.angle))
                );
            }
            if (body.transformedVertices.length === 0) {
                 body.aabb.min = Vec2.clone(body.position);
                 body.aabb.max = Vec2.clone(body.position);
                 return;
            }

            let minX = Infinity, minY = Infinity;
            let maxX = -Infinity, maxY = -Infinity;

            for (const v of body.transformedVertices) {
                if (v.x < minX) minX = v.x;
                if (v.x > maxX) maxX = v.x;
                if (v.y < minY) minY = v.y;
                if (v.y > maxY) maxY = v.y;
            }
            body.aabb.min = Vec2.create(minX, minY);
            body.aabb.max = Vec2.create(maxX, maxY);

        } else if (body.shapeType === 'CIRCLE') {
            body.aabb.min = Vec2.create(body.position.x - body.radius, body.position.y - body.radius);
            body.aabb.max = Vec2.create(body.position.x + body.radius, body.position.y + body.radius);
        }
    } else { // Should not happen if shapeType is validated at creation
        body.aabb.min = Vec2.clone(body.position);
        body.aabb.max = Vec2.clone(body.position);
    }
}


// New function for Broad Phase Collision Detection
function broadPhaseCheck(bodies) {
    const potentialPairs = [];

    for (let i = 0; i < bodies.length; i++) {
        const bodyA = bodies[i];
        // Update AABB for bodyA before checks if it's not done globally each frame yet
        // For now, assume AABBs are updated prior to calling broadPhaseCheck,
        // e.g., after all bodies have their positions/angles updated.
        // updateAABB(bodyA);

        for (let j = i + 1; j < bodies.length; j++) {
            const bodyB = bodies[j];
            // updateAABB(bodyB);

            // Optimization: Skip pairs of two static bodies
            if (bodyA.isStatic && bodyB.isStatic) {
                continue;
            }

            // AABB intersection test
            if (bodyA.aabb.max.x > bodyB.aabb.min.x &&
                bodyA.aabb.min.x < bodyB.aabb.max.x &&
                bodyA.aabb.max.y > bodyB.aabb.min.y &&
                bodyA.aabb.min.y < bodyB.aabb.max.y) {
                potentialPairs.push({ bodyA, bodyB });
            }
        }
    }
    // console.log(`[BroadPhase] Found ${potentialPairs.length} potential collision pairs.`);
    return potentialPairs;
}

    // Helper function to calculate mass properties for a convex polygon
    // Assumes vertices are ordered (e.g., clockwise or counter-clockwise)
    // And relative to the desired center of mass (or this function can center them)
    function calculatePolygonProperties(vertices, density) {
        if (vertices.length < 3) {
            return { mass: 0, inertia: 0, area: 0, geometricCenter: Vec2.create() };
        }

        let area = 0;
        let cx = 0; // For centroid x
        let cy = 0; // For centroid y

        // Calculate area and centroid (geometric center) using Shoelace formula
        for (let i = 0; i < vertices.length; i++) {
            const v1 = vertices[i];
            const v2 = vertices[(i + 1) % vertices.length]; // Next vertex, wraps around

            const crossProduct = Vec2.crossScalar(v1, v2);
            area += crossProduct;
            cx += (v1.x + v2.x) * crossProduct;
            cy += (v1.y + v2.y) * crossProduct;
        }
        area *= 0.5;

        if (Math.abs(area) < 1e-9) { // Check for zero or very small area
            console.warn("[calculatePolygonProperties] Polygon area is zero or very small. Vertices might be collinear or duplicated.", vertices);
            return { mass: 0, inertia: 0, area: 0, geometricCenter: Vec2.create(), centeredVertices: vertices.map(v => Vec2.clone(v)) };
        }

        const geometricCenter = Vec2.create(cx / (6 * area), cy / (6 * area));
        const centeredVertices = vertices.map(v => Vec2.sub(v, geometricCenter));
        const mass = Math.abs(area) * density;

        let numerator = 0;
        for (let i = 0; i < centeredVertices.length; i++) {
            const v_i = centeredVertices[i];
            const v_i_plus_1 = centeredVertices[(i + 1) % centeredVertices.length];
            numerator += Vec2.crossScalar(v_i, v_i_plus_1) *
                         (Vec2.dot(v_i, v_i) + Vec2.dot(v_i_plus_1, v_i_plus_1) + Vec2.dot(v_i, v_i_plus_1));
        }

        let denominator_sum_cross = 0;
         for (let i = 0; i < centeredVertices.length; i++) {
            denominator_sum_cross += Vec2.crossScalar(centeredVertices[i], centeredVertices[(i + 1) % centeredVertices.length]);
        }
        // denominator_sum_cross is 2 * signed area of the centered polygon.

        let finalInertia = 0;
        if (Math.abs(denominator_sum_cross) > 1e-9) {
             finalInertia = (numerator / denominator_sum_cross) * (mass / 6);
        } else {
            console.warn("[calculatePolygonProperties] Denominator for inertia calculation is zero or very small (polygon area issue). Setting inertia to 0.", centeredVertices);
        }

        return {
            mass: mass,
            inertia: Math.abs(finalInertia),
            area: Math.abs(area),
            geometricCenter: geometricCenter,
            centeredVertices: centeredVertices
        };
    }

    // --- Old PDS Code - To Be Removed or Refactored ---

    /*
    // Old Matter.js based component definitions
    const componentDefaults = {
        chassis: { /* ... */ },
        wheel: { /* ... */ },
        spike: { /* ... */ }
    };

    // Old Matter.js based gameState
    const gameState = {
        engine: null,
        render: null,
        world: null,
        mode: 'ASSEMBLY',
        canvas: null,
        canvasWidth: 800,
        canvasHeight: 600,
        playerRobot: { parts: [], constraints: [] },
        enemyRobot: { parts: [], constraints: [] },
        dragging: { /* ... */ },
        playerControls: { /* ... */ },
        ui: { /* ... */ },
        // ... other old gameState properties
    };

    // Old Matter.js Aliases
    // const Engine = Matter.Engine;
    // const Render = Matter.Render;
    // const Runner = Matter.Runner;
    // const World = Matter.World;
    // const Bodies = Matter.Bodies;
    // const Composite = Matter.Composite;
    // const Events = Matter.Events;
    // const Constraint = Matter.Constraint;

    // Constants from old implementation
    // const PLAYER_TORQUE_AMOUNT = 0.15;
    // const AI_TORQUE_AMOUNT = 0.08;
    // const BASE_DAMAGE_MULTIPLIER = 2;
    // const MIN_COLLISION_SPEED_FOR_DAMAGE = 2;
    // const SNAP_RADIUS = 30;

    // All other functions from the previous Matter.js implementation
    // are considered commented out from this point. Examples:
    // function handlePlayerInput(event, isKeyDown) { ... }
    // function applyPlayerControls() { ... }
    // function checkSnapPoints(event) { ... }
    // function clearSnapState() { ... }
    // function renderSnapHighlights() { ... }
    // function drawConnectionPoints(event) { ... }
    // function initEngine() { ... }
    // function initControls() { ... }
    // function initComponents() { ... }
    // function getMousePosInCanvas(event) { ... }
    // function initiateDrag(event, partType) { ... }
    // function moveGhostElement(event) { ... }
    // function handleDragMove(event) { ... }
    // function handleDragEnd(event) { ... }
    // function createPartOnCanvas(partType, x, y, angle, snappedTo) { ... }
    // function createEnemyRobot(x, y) { ... }
    // function spawnRobots() { ... }
    // function clearBattlefield() { ... }
    // function startBattle() { ... }
    // function resetToEditor() { ... }
    // function handleCollisions(pairs) { ... }
    // function handleGameOver(winner) { ... }
    // function checkAndProcessDestruction() { ... }
    */

    // Modify main() to use the debug renderer
    function main() {
        console.log("[NEW_PDS_INIT] main() called. Project Advanced PDS started.");

        const bodies = [];
        const ground = createRigidBody({
            position: Vec2.create(20, 25),
            shapeType: 'POLYGON',
            vertices: [Vec2.create(-20, -1), Vec2.create(20, -1), Vec2.create(20, 1), Vec2.create(-20, 1)],
            isStatic: true, restitution: 0.5
        });
        bodies.push(ground);

        const box1 = createRigidBody({
            position: Vec2.create(18, 5),
            shapeType: 'POLYGON',
            vertices: [Vec2.create(-1, -1), Vec2.create(1, -1), Vec2.create(1, 1), Vec2.create(-1, 1)],
            mass: 1, inertia: (1 * (2*2 + 2*2))/12, restitution: 0.3
        });
        bodies.push(box1);

        const box2 = createRigidBody({
            position: Vec2.create(22, 10), // Changed initial y to be higher for more falling
            angle: Math.PI / 6,
            shapeType: 'POLYGON',
            vertices: [Vec2.create(-1.5, -0.5), Vec2.create(1.5, -0.5), Vec2.create(1.5, 0.5), Vec2.create(-1.5, 0.5)],
            mass: 2, inertia: (2 * (3*3 + 1*1))/12, restitution: 0.3
        });
        bodies.push(box2);

        const deltaTime = 1/60;
        const numSteps = 60; // Simulate for 1 second

        console.log("--- Initial Body States (for textual render) ---");
        console.log(debugRenderWorldToString(bodies, [], 0));


        for (let i = 0; i < numSteps; i++) {
            bodies.forEach(body => { if(!body.isStatic) updateTransformedVertices(body); });
            const potentialPairs = broadPhaseCheck(bodies);
            const actualCollisions = narrowPhaseCheck(potentialPairs);

            physicsStep(deltaTime, bodies, actualCollisions);

            if ((i + 1) % 15 === 0) { // Log debug render every 15 steps (4 times per second for 60fps)
                console.log(debugRenderWorldToString(bodies, actualCollisions, i + 1));
            }
        }
        console.log("--- Final Body States (for textual render) ---");
        console.log(debugRenderWorldToString(bodies, narrowPhaseCheck(broadPhaseCheck(bodies)), numSteps)); // Render final state with collisions
    }

// --- Basic Physics Debug Renderer (Textual) ---

function debugRenderWorldToString(bodies, collisions, stepNumber) {
    let output = `--- Debug Render: Step ${stepNumber} ---\n`;
    output += `Bodies (${bodies.length}):\n`;

    for (const body of bodies) {
        output += `  Body ID: ${body.id} (${body.shapeType}, ${body.isStatic ? 'Static' : 'Dynamic'})
`;
        output += `    Position: (${body.position.x.toFixed(2)}, ${body.position.y.toFixed(2)})
`;
        output += `    Angle: ${body.angle.toFixed(2)} rad
`;
        output += `    LinVel: (${body.linearVelocity.x.toFixed(2)}, ${body.linearVelocity.y.toFixed(2)})
`;
        output += `    AngVel: ${body.angularVelocity.toFixed(2)} rad/s
`;
        output += `    AABB: min(${body.aabb.min.x.toFixed(2)},${body.aabb.min.y.toFixed(2)}) max(${body.aabb.max.x.toFixed(2)},${body.aabb.max.y.toFixed(2)})
`;

        if (body.shapeType === 'POLYGON' && body.transformedVertices.length > 0) {
            output += `    Vertices (World): ${body.transformedVertices.map(v => `(${v.x.toFixed(2)},${v.y.toFixed(2)})`).join(' ')}
`;
        } else if (body.shapeType === 'CIRCLE') {
            output += `    Circle: Center=(${body.position.x.toFixed(2)},${body.position.y.toFixed(2)}), Radius=${body.radius.toFixed(2)}
`;
        }
        output += '\n';
    }

    if (collisions && collisions.length > 0) {
        output += `Collisions (${collisions.length}):\n`;
        for (const collision of collisions) {
            output += `  Collision: Body ${collision.bodyA.id} vs Body ${collision.bodyB.id}
`;
            output += `    Normal: (${collision.normal.x.toFixed(2)}, ${collision.normal.y.toFixed(2)})
`;
            output += `    Depth: ${collision.depth.toFixed(2)}
`;
            output += '\n';
        }
    } else {
        output += "No collisions detected in this step.\n";
    }
    output += "--- End Debug Render ---\n";
    return output;
}

// --- Collision Response & Dynamics Simulation ---

const CORRECTION_PERCENT = 0.4; // How much of the penetration to correct per step (0.2 to 0.8 is common)
const SLOP = 0.01; // Penetration allowance to prevent jittering (small value)

function resolveCollision(collisionInfo) {
    const { bodyA, bodyB, normal, depth } = collisionInfo;

    // --- 1. Penetration Resolution (Position Correction) ---
    const penetrationToCorrect = Math.max(0, depth - SLOP);
    if (penetrationToCorrect > 0) {
        const totalInvMass = bodyA.invMass + bodyB.invMass;
        if (totalInvMass === 0) return; // Both bodies are static or have infinite mass

        const correctionAmount = penetrationToCorrect / totalInvMass * CORRECTION_PERCENT;
        const correctionVector = Vec2.scale(normal, correctionAmount);

        if (!bodyA.isStatic) {
            bodyA.position = Vec2.sub(bodyA.position, Vec2.scale(correctionVector, bodyA.invMass));
            updateTransformedVertices(bodyA); // AABB will also be updated
        }
        if (!bodyB.isStatic) {
            bodyB.position = Vec2.add(bodyB.position, Vec2.scale(correctionVector, bodyB.invMass));
            updateTransformedVertices(bodyB); // AABB will also be updated
        }
        // console.log(`[Resolve] Corrected penetration by ${penetrationToCorrect.toFixed(3)} between ${bodyA.id} and ${bodyB.id}`);
    }

    // --- 2. Relative Velocity (simplified: between CoMs) ---
    const rv = Vec2.sub(bodyB.linearVelocity, bodyA.linearVelocity);

    // --- 3. Calculate Impulse Magnitude (J) (simplified: no rotation, no contact points) ---
    const rvDotNormal = Vec2.dot(rv, normal);

    // If velocities are separating, do not apply impulse (unless it's a very small separation post-correction)
    if (rvDotNormal > 0 && depth < SLOP * 2) { // Allow small impulse if they are still very close after correction
        // console.log(`[Resolve] Velocities separating for ${bodyA.id}-${bodyB.id}. rvDotNormal: ${rvDotNormal.toFixed(3)}`);
        return;
    }

    const e = Math.min(bodyA.restitution, bodyB.restitution); // Coefficient of restitution

    let J_numerator = -(1 + e) * rvDotNormal;
    let J_denominator = bodyA.invMass + bodyB.invMass;

    if (J_denominator === 0) return; // Both static, no impulse

    const J = J_numerator / J_denominator;

    // --- 4. Apply Linear Impulse ---
    const impulseVector = Vec2.scale(normal, J);

    if (!bodyA.isStatic) {
        bodyA.linearVelocity = Vec2.sub(bodyA.linearVelocity, Vec2.scale(impulseVector, bodyA.invMass));
    }
    if (!bodyB.isStatic) {
        bodyB.linearVelocity = Vec2.add(bodyB.linearVelocity, Vec2.scale(impulseVector, bodyB.invMass));
    }

    // console.log(`[Resolve] Applied impulse ${J.toFixed(3)} to ${bodyA.id} and ${bodyB.id}. RV.N: ${rvDotNormal.toFixed(3)}`);

    // TODO: Friction impulse would be handled here
}


// --- Physics Step (Integrator) ---
const GRAVITY = Vec2.create(0, 9.81); // Example gravity (adjust strength as needed, e.g., 98.1 for pixels/sec^2)
                                     // PDS Sec 3.1: engine.world.gravity.y = 1.0 (standard earth-like gravity)
                                     // This implies units are meters/sec^2 if objects are in meters.
                                     // If units are pixels, gravity might be smaller, e.g. 0.1 to 1.0 pixels/step^2
                                     // Let's use a smaller value for pixel-based simulation initially.
const PIXEL_GRAVITY = Vec2.create(0, 1); // Smaller gravity suitable for pixel units / larger masses

function physicsStep(deltaTime, bodies, allCollisions) {
    // 1. Apply forces (e.g., gravity) & Integrate velocities
    for (const body of bodies) {
        if (body.isStatic) continue;

        // Apply gravity
        const gravityForce = Vec2.scale(PIXEL_GRAVITY, body.mass); // F = m*g
        body.force = Vec2.add(body.force, gravityForce);

        // Integrate linear velocity (Explicit Euler)
        // v = v + (F/m) * dt
        body.linearVelocity = Vec2.add(body.linearVelocity, Vec2.scale(body.force, body.invMass * deltaTime));

        // Integrate angular velocity (Explicit Euler)
        // w = w + (T/I) * dt
        body.angularVelocity += body.torque * body.invInertia * deltaTime;

        // TODO: Add damping if needed (linear and angular)
    }

    // 2. Resolve collisions (iteratively if using sequential impulses, or all at once)
    // For simplicity, one pass of resolution. Multiple passes can improve stability.
    for (const collision of allCollisions) {
        resolveCollision(collision);
    }

    // 3. Integrate positions & Update orientation
    for (const body of bodies) {
        if (body.isStatic) continue;

        body.position = Vec2.add(body.position, Vec2.scale(body.linearVelocity, deltaTime));
        body.angle += body.angularVelocity * deltaTime;

        // 4. Update transformed vertices and AABB for collision detection in next frame
        updateTransformedVertices(body); // This also calls updateAABB

        // 5. Reset accumulators
        body.force = Vec2.create(0, 0);
        body.torque = 0;
    }
}


// --- Game Loop (Conceptual) ---
// let lastTime = 0;
// const allBodiesInSim = []; // Populate this with your rigid bodies
// function gameLoop(currentTime) {
//     const deltaTime = (currentTime - lastTime) / 1000; // Delta time in seconds
//     lastTime = currentTime;

//     // 1. Input handling (already done elsewhere for player)
//     // 2. AI Update (already done elsewhere)
//     // 3. Broad phase
//     const potentialPairs = broadPhaseCheck(allBodiesInSim);
//     // 4. Narrow phase
//     const actualCollisions = narrowPhaseCheck(potentialPairs);
//     // 5. Physics step (integrator + collision response)
//     physicsStep(deltaTime, allBodiesInSim, actualCollisions);
//     // 6. Rendering (done by a debug renderer or game renderer)
//     // renderDebug(allBodiesInSim, actualCollisions);

//     requestAnimationFrame(gameLoop);
// }
// To start: lastTime = performance.now(); requestAnimationFrame(gameLoop);

// --- Narrow Phase Collision Detection ---

// Helper to get unique edge normals of a polygon
function getAxes(vertices) {
    const axes = [];
    for (let i = 0; i < vertices.length; i++) {
        const p1 = vertices[i];
        const p2 = vertices[(i + 1) % vertices.length]; // Next vertex
        const edge = Vec2.sub(p2, p1);
        const normal = Vec2.normalize(Vec2.perpendicular(edge));
        // Add normal only if it's not a duplicate (e.g. for axis-aligned boxes)
        // For simplicity now, we might get duplicate axes for rectangles, but SAT handles it.
        // A more robust way is to check if axis or its negation is already present.
        axes.push(normal);
    }
    return axes;
}

// Helper to project polygon vertices onto an axis
function projectPolygon(axis, vertices) {
    let min = Vec2.dot(axis, vertices[0]);
    let max = min;
    for (let i = 1; i < vertices.length; i++) {
        const projection = Vec2.dot(axis, vertices[i]);
        if (projection < min) {
            min = projection;
        } else if (projection > max) {
            max = projection;
        }
    }
    return { min, max };
}

// SAT check for two polygons
// Returns null if no collision, or an object with { normal, depth } if collision
function checkPolygonPolygonCollision(polyA, polyB) {
    // Use transformedVertices which are in world space
    const verticesA = polyA.transformedVertices;
    const verticesB = polyB.transformedVertices;

    if (verticesA.length < 3 || verticesB.length < 3) return null; // Not valid polygons

    let minOverlap = Infinity;
    let smallestAxis = null;

    // Get axes from both polygons
    const axesA = getAxes(verticesA);
    const axesB = getAxes(verticesB);
    const allAxes = [...axesA, ...axesB];

    for (const axis of allAxes) {
        const projectionA = projectPolygon(axis, verticesA);
        const projectionB = projectPolygon(axis, verticesB);

        // Check for separation on this axis
        const overlap = Math.min(projectionA.max, projectionB.max) - Math.max(projectionA.min, projectionB.min);
        if (overlap <= 0) { // Found a separating axis
            return null; // No collision
        }

        // If overlap is the smallest so far, store it and the axis
        if (overlap < minOverlap) {
            minOverlap = overlap;
            smallestAxis = axis;
        }
    }

    // If we reach here, polygons are colliding.
    // The smallestAxis is the axis of minimum penetration.
    // Ensure the normal points from A to B (or consistently).
    // Calculate the vector from center of A to center of B
    const directionVector = Vec2.sub(polyB.position, polyA.position);
    if (Vec2.dot(directionVector, smallestAxis) < 0) {
        // If the normal is pointing from B to A, flip it
        smallestAxis = Vec2.negate(smallestAxis);
    }

    // TODO: Contact point calculation is complex and deferred.
    // For now, we only have normal and depth.

    return {
        bodyA: polyA,
        bodyB: polyB,
        normal: smallestAxis, // Unit vector from A into B ideally
        depth: minOverlap,
        contactPoints: [] // Placeholder
    };
}


function narrowPhaseCheck(potentialPairs) {
    const collisions = [];
    for (const pair of potentialPairs) {
        const { bodyA, bodyB } = pair;
        let collisionInfo = null;

        // TODO: Add Circle-Circle and Polygon-Circle checks
        if (bodyA.shapeType === 'POLYGON' && bodyB.shapeType === 'POLYGON') {
            collisionInfo = checkPolygonPolygonCollision(bodyA, bodyB);
        } else if (bodyA.shapeType === 'CIRCLE' && bodyB.shapeType === 'CIRCLE') {
            // collisionInfo = checkCircleCircleCollision(bodyA, bodyB); // To be implemented
            console.warn("[NarrowPhase] Circle-Circle collision check not yet implemented.");
        } else if (bodyA.shapeType === 'POLYGON' && bodyB.shapeType === 'CIRCLE') {
            // collisionInfo = checkPolygonCircleCollision(bodyA, bodyB); // Order matters for implementation
            console.warn("[NarrowPhase] Polygon-Circle collision check not yet implemented.");
        } else if (bodyA.shapeType === 'CIRCLE' && bodyB.shapeType === 'POLYGON') {
            // collisionInfo = checkPolygonCircleCollision(bodyB, bodyA); // Reuse by swapping order
            console.warn("[NarrowPhase] Circle-Polygon collision check not yet implemented.");
        }


        if (collisionInfo) {
            collisions.push(collisionInfo);
        }
    }
    return collisions;
}

    document.addEventListener('DOMContentLoaded', main);

})(); // End of IIFE
