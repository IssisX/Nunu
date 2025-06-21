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

        crossScalar: (v1, v2) => (v1.x * v2.y) - (v1.y * v2.x),

        magnitudeSq: (v) => v.x * v.x + v.y * v.y,
        magnitude: (v) => Math.sqrt(v.x * v.x + v.y * v.y),

        normalize: (v) => {
            const mag = Vec2.magnitude(v);
            if (mag === 0) return Vec2.create(0, 0);
            return Vec2.invScale(v, mag);
        },

        distanceSq: (v1, v2) => {
            const dx = v1.x - v2.x;
            const dy = v1.y - v2.y;
            return dx * dx + dy * dy;
        },
        distance: (v1, v2) => Math.sqrt(Vec2.distanceSq(v1, v2)),

        rotate: (v, angleRad) => {
            const cosA = Math.cos(angleRad);
            const sinA = Math.sin(angleRad);
            return Vec2.create(
                v.x * cosA - v.y * sinA,
                v.x * sinA + v.y * cosA
            );
        },

        perpendicular: (v) => Vec2.create(-v.y, v.x),

        lerp: (v1, v2, t) => Vec2.add(v1, Vec2.scale(Vec2.sub(v2, v1), t)),

        negate: (v) => Vec2.create(-v.x, -v.y),

        project: (v1, v2) => {
            const magSqV2 = Vec2.magnitudeSq(v2);
            if (magSqV2 === 0) return Vec2.create(0,0);
            const dotProduct = Vec2.dot(v1, v2);
            return Vec2.scale(v2, dotProduct / magSqV2);
        }
    };
    console.log('[NEW_PDS_INIT] Vec2 math library defined.');

    let nextBodyId = 0;

    function createRigidBody(config) {
        const body = {
            id: nextBodyId++,
            position: config.position ? Vec2.clone(config.position) : Vec2.create(0, 0),
            angle: config.angle || 0,
            linearVelocity: config.linearVelocity ? Vec2.clone(config.linearVelocity) : Vec2.create(0, 0),
            angularVelocity: config.angularVelocity || 0,
            force: Vec2.create(0, 0),
            torque: 0,
            mass: config.mass || 1,
            invMass: 0,
            inertia: config.inertia || 1,
            invInertia: 0,
            shapeType: config.shapeType || 'POLYGON',
            vertices: [],
            transformedVertices: [],
            radius: config.radius || 0,
            restitution: config.restitution || 0.2,
            staticFriction: config.staticFriction || 0.5,
            dynamicFriction: config.dynamicFriction || 0.3,
            materialId: config.materialId || 'default_material',
            isStatic: config.isStatic || false,
            aabb: { min: Vec2.create(), max: Vec2.create() },
        };

        if (body.isStatic) {
            body.invMass = 0;
            body.invInertia = 0;
            body.mass = Infinity;
            body.inertia = Infinity;
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
            body.vertices = config.vertices.map(v => Vec2.clone(v));
        } else if (body.shapeType === 'CIRCLE') {
            if (body.radius <= 0) {
                throw new Error('Circle shape requires a positive radius.');
            }
        } else {
            throw new Error(`Unsupported shapeType: ${body.shapeType}`);
        }

        updateTransformedVertices(body);
        console.log(`[RigidBody] Created ${body.isStatic ? 'static' : 'dynamic'} ${body.shapeType} body ID ${body.id}. AABB: min(${body.aabb.min.x.toFixed(1)},${body.aabb.min.y.toFixed(1)}), max(${body.aabb.max.x.toFixed(1)},${body.aabb.max.y.toFixed(1)})`);
        return body;
    }

    function updateTransformedVertices(body) {
        if (body.shapeType === 'POLYGON') {
            body.transformedVertices = body.vertices.map(vertex =>
                Vec2.add(body.position, Vec2.rotate(vertex, body.angle))
            );
        }
        updateAABB(body);
    }

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
        } else {
            body.aabb.min = Vec2.clone(body.position);
            body.aabb.max = Vec2.clone(body.position);
        }
    }

    function calculatePolygonProperties(vertices, density) {
        if (vertices.length < 3) {
            return { mass: 0, inertia: 0, area: 0, geometricCenter: Vec2.create(), centeredVertices: [] };
        }
        let area = 0;
        let cx = 0;
        let cy = 0;
        for (let i = 0; i < vertices.length; i++) {
            const v1 = vertices[i];
            const v2 = vertices[(i + 1) % vertices.length];
            const crossProduct = Vec2.crossScalar(v1, v2);
            area += crossProduct;
            cx += (v1.x + v2.x) * crossProduct;
            cy += (v1.y + v2.y) * crossProduct;
        }
        area *= 0.5;
        if (Math.abs(area) < 1e-9) {
            console.warn("[calculatePolygonProperties] Polygon area is zero or very small.", vertices);
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
        let finalInertia = 0;
        if (Math.abs(denominator_sum_cross) > 1e-9) {
             finalInertia = (numerator / denominator_sum_cross) * (mass / 6);
        } else {
            console.warn("[calculatePolygonProperties] Denominator for inertia calculation is zero (polygon area issue). Inertia set to 0.", centeredVertices);
        }
        return {
            mass: mass, inertia: Math.abs(finalInertia), area: Math.abs(area),
            geometricCenter: geometricCenter, centeredVertices: centeredVertices
        };
    }

    function broadPhaseCheck(bodies) {
        const potentialPairs = [];
        for (let i = 0; i < bodies.length; i++) {
            const bodyA = bodies[i];
            for (let j = i + 1; j < bodies.length; j++) {
                const bodyB = bodies[j];
                if (bodyA.isStatic && bodyB.isStatic) continue;
                if (bodyA.aabb.max.x > bodyB.aabb.min.x &&
                    bodyA.aabb.min.x < bodyB.aabb.max.x &&
                    bodyA.aabb.max.y > bodyB.aabb.min.y &&
                    bodyA.aabb.min.y < bodyB.aabb.max.y) {
                    potentialPairs.push({ bodyA, bodyB });
                }
            }
        }
        return potentialPairs;
    }

    function getAxes(vertices) {
        const axes = [];
        for (let i = 0; i < vertices.length; i++) {
            const p1 = vertices[i];
            const p2 = vertices[(i + 1) % vertices.length];
            const edge = Vec2.sub(p2, p1);
            axes.push(Vec2.normalize(Vec2.perpendicular(edge)));
        }
        return axes;
    }

    function projectPolygon(axis, vertices) {
        let min = Vec2.dot(axis, vertices[0]);
        let max = min;
        for (let i = 1; i < vertices.length; i++) {
            const projection = Vec2.dot(axis, vertices[i]);
            if (projection < min) min = projection;
            else if (projection > max) max = projection;
        }
        return { min, max };
    }

    function checkPolygonPolygonCollision(polyA, polyB) {
        const verticesA = polyA.transformedVertices;
        const verticesB = polyB.transformedVertices;
        if (verticesA.length < 3 || verticesB.length < 3) return null;
        let minOverlap = Infinity;
        let smallestAxis = null;
        const allAxes = [...getAxes(verticesA), ...getAxes(verticesB)];
        for (const axis of allAxes) {
            const projectionA = projectPolygon(axis, verticesA);
            const projectionB = projectPolygon(axis, verticesB);
            const overlap = Math.min(projectionA.max, projectionB.max) - Math.max(projectionA.min, projectionB.min);
            if (overlap <= 0) return null;
            if (overlap < minOverlap) {
                minOverlap = overlap;
                smallestAxis = axis;
            }
        }
        const directionVector = Vec2.sub(polyB.position, polyA.position);
        if (Vec2.dot(directionVector, smallestAxis) < 0) {
            smallestAxis = Vec2.negate(smallestAxis);
        }
        return {
            bodyA: polyA, bodyB: polyB,
            normal: smallestAxis, depth: minOverlap,
            contactPoints: []
        };
    }

    function narrowPhaseCheck(potentialPairs) {
        const collisions = [];
        for (const pair of potentialPairs) {
            const { bodyA, bodyB } = pair;
            let collisionInfo = null;
            if (bodyA.shapeType === 'POLYGON' && bodyB.shapeType === 'POLYGON') {
                collisionInfo = checkPolygonPolygonCollision(bodyA, bodyB);
            } else if (bodyA.shapeType === 'CIRCLE' && bodyB.shapeType === 'CIRCLE') {
                console.warn("[NarrowPhase] Circle-Circle collision check not yet implemented.");
            } else if ((bodyA.shapeType === 'POLYGON' && bodyB.shapeType === 'CIRCLE') ||
                       (bodyA.shapeType === 'CIRCLE' && bodyB.shapeType === 'POLYGON')) {
                console.warn("[NarrowPhase] Polygon-Circle/Circle-Polygon collision check not yet implemented.");
            }
            if (collisionInfo) collisions.push(collisionInfo);
        }
        return collisions;
    }

    const CORRECTION_PERCENT = 0.4;
    const SLOP = 0.01;
    const PIXEL_GRAVITY = Vec2.create(0, 1);

    function resolveCollision(collisionInfo) {
        const { bodyA, bodyB, normal, depth } = collisionInfo;
        const penetrationToCorrect = Math.max(0, depth - SLOP);
        if (penetrationToCorrect > 0) {
            const totalInvMass = bodyA.invMass + bodyB.invMass;
            if (totalInvMass > 0) { // Avoid division by zero if both are static
                const correctionAmount = penetrationToCorrect / totalInvMass * CORRECTION_PERCENT;
                const correctionVector = Vec2.scale(normal, correctionAmount);
                if (!bodyA.isStatic) {
                    bodyA.position = Vec2.sub(bodyA.position, Vec2.scale(correctionVector, bodyA.invMass));
                    updateTransformedVertices(bodyA);
                }
                if (!bodyB.isStatic) {
                    bodyB.position = Vec2.add(bodyB.position, Vec2.scale(correctionVector, bodyB.invMass));
                    updateTransformedVertices(bodyB);
                }
            }
        }
        const rv = Vec2.sub(bodyB.linearVelocity, bodyA.linearVelocity);
        const rvDotNormal = Vec2.dot(rv, normal);
        if (rvDotNormal > 0 && depth < SLOP * 2) return;
        const e = Math.min(bodyA.restitution, bodyB.restitution);
        let J_numerator = -(1 + e) * rvDotNormal;
        let J_denominator = bodyA.invMass + bodyB.invMass;
        if (J_denominator === 0) return;
        const J = J_numerator / J_denominator;
        const impulseVector = Vec2.scale(normal, J);
        if (!bodyA.isStatic) {
            bodyA.linearVelocity = Vec2.sub(bodyA.linearVelocity, Vec2.scale(impulseVector, bodyA.invMass));
        }
        if (!bodyB.isStatic) {
            bodyB.linearVelocity = Vec2.add(bodyB.linearVelocity, Vec2.scale(impulseVector, bodyB.invMass));
        }
    }

    function physicsStep(deltaTime, bodies, allCollisions) {
        for (const body of bodies) {
            if (body.isStatic) continue;
            const gravityForce = Vec2.scale(PIXEL_GRAVITY, body.mass);
            body.force = Vec2.add(body.force, gravityForce);
            body.linearVelocity = Vec2.add(body.linearVelocity, Vec2.scale(body.force, body.invMass * deltaTime));
            body.angularVelocity += body.torque * body.invInertia * deltaTime;
        }
        for (const collision of allCollisions) {
            resolveCollision(collision);
        }
        for (const body of bodies) {
            if (body.isStatic) continue;
            body.position = Vec2.add(body.position, Vec2.scale(body.linearVelocity, deltaTime));
            body.angle += body.angularVelocity * deltaTime;
            updateTransformedVertices(body);
            body.force = Vec2.create(0, 0);
            body.torque = 0;
        }
    }

    function debugRenderWorldToString(bodies, collisions, stepNumber) {
        let output = `--- Debug Render: Step ${stepNumber} ---\n`;
        output += `Bodies (${bodies.length}):\n`;
        for (const body of bodies) {
            output += `  Body ID: ${body.id} (${body.shapeType}, ${body.isStatic ? 'Static' : 'Dynamic'})\n`;
            output += `    Position: (${body.position.x.toFixed(2)}, ${body.position.y.toFixed(2)})\n`;
            output += `    Angle: ${body.angle.toFixed(2)} rad\n`;
            output += `    LinVel: (${body.linearVelocity.x.toFixed(2)}, ${body.linearVelocity.y.toFixed(2)})\n`;
            output += `    AngVel: ${body.angularVelocity.toFixed(2)} rad/s\n`;
            output += `    AABB: min(${body.aabb.min.x.toFixed(2)},${body.aabb.min.y.toFixed(2)}) max(${body.aabb.max.x.toFixed(2)},${body.aabb.max.y.toFixed(2)})\n`;
            if (body.shapeType === 'POLYGON' && body.transformedVertices.length > 0) {
                output += `    Vertices (World): ${body.transformedVertices.map(v => `(${v.x.toFixed(2)},${v.y.toFixed(2)})`).join(' ')}\n`;
            } else if (body.shapeType === 'CIRCLE') {
                output += `    Circle: Center=(${body.position.x.toFixed(2)},${body.position.y.toFixed(2)}), Radius=${body.radius.toFixed(2)}\n`;
            }
            output += '\n';
        }
        if (collisions && collisions.length > 0) {
            output += `Collisions (${collisions.length}):\n`;
            for (const collision of collisions) {
                output += `  Collision: Body ${collision.bodyA.id} vs Body ${collision.bodyB.id}\n`;
                output += `    Normal: (${collision.normal.x.toFixed(2)}, ${collision.normal.y.toFixed(2)})\n`;
                output += `    Depth: ${collision.depth.toFixed(2)}\n\n`;
            }
        } else {
            output += "No collisions detected in this step.\n";
        }
        output += "--- End Debug Render ---\n";
        return output;
    }

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
            position: Vec2.create(22, 10),
            angle: Math.PI / 6,
            shapeType: 'POLYGON',
            vertices: [Vec2.create(-1.5, -0.5), Vec2.create(1.5, -0.5), Vec2.create(1.5, 0.5), Vec2.create(-1.5, 0.5)],
            mass: 2, inertia: (2 * (3*3 + 1*1))/12, restitution: 0.3
        });
        bodies.push(box2);

        const deltaTime = 1/60;
        const numSteps = 60;
        console.log("--- Initial Body States (for textual render) ---");
        console.log(debugRenderWorldToString(bodies, [], 0));
        for (let i = 0; i < numSteps; i++) {
            bodies.forEach(body => { if(!body.isStatic) updateTransformedVertices(body); });
            const potentialPairs = broadPhaseCheck(bodies);
            const actualCollisions = narrowPhaseCheck(potentialPairs);
            physicsStep(deltaTime, bodies, actualCollisions);
            if ((i + 1) % 15 === 0) {
                console.log(debugRenderWorldToString(bodies, actualCollisions, i + 1));
            }
        }
        console.log("--- Final Body States (for textual render) ---");
        console.log(debugRenderWorldToString(bodies, narrowPhaseCheck(broadPhaseCheck(bodies)), numSteps));
    }

    document.addEventListener('DOMContentLoaded', main);

})(); // End of IIFE
