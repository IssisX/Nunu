// PDS 5.2: IIFE to encapsulate the game logic
(() => {
    // Add this before the gameState object definition

    const componentDefaults = {
        chassis: {
            label: 'chassis',
            health: 250,
            mass: 10, // PDS 3.2
            width: 100, // Default visual size
            height: 80, // Default visual size
            connectionPoints: [ // Relative to center of chassis
                { x: 0, y: -40, type: 'top' },    // Top
                { x: 0, y: 40, type: 'bottom' }, // Bottom
                { x: -50, y: 0, type: 'left' },   // Left
                { x: 50, y: 0, type: 'right' }    // Right
            ],
            keyMechanic: 'Core component. Destruction = loss.',
            render: {
                fillStyle: '#8A8A8A', // Medium Grey
                strokeStyle: '#505050',
                lineWidth: 2
            }
        },
        wheel: {
            label: 'wheel',
            health: 80,
            mass: 5, // PDS 3.2
            radius: 25, // Default visual size
            connectionPoints: [], // Wheels don't typically have points *for others* to connect to
            keyMechanic: 'Provides motion. friction: 0.8. torque applied via controls.',
            matterJsOptions: {
                friction: 0.8 // Specific to wheel physics
            },
            render: {
                fillStyle: '#505050', // Dark Grey
                strokeStyle: '#303030',
                lineWidth: 2
            }
        },
        spike: {
            label: 'spike',
            health: 50,
            mass: 3, // PDS 3.2
            width: 20,  // Base width
            height: 60, // Length of spike
            connectionPoints: [ // Base of the spike is the connection point
                { x: 0, y: 30, type: 'base' } // Connects at its bottom
            ],
            keyMechanic: 'Primary weapon. Deals damage on collision.',
            render: {
                fillStyle: '#D32F2F', // Red for weapon
                strokeStyle: '#A02020',
                lineWidth: 2
            },
            vertices: [ // Custom shape for spike (triangle)
                { x: 0, y: -30 }, { x: -10, y: 30 }, { x: 10, y: 30 }
            ]
        }
        // Future parts can be added here
    };

    // Make sure gameState is defined after componentDefaults
    // PDS 5.3: Centralized gameState object
    const gameState = {
        engine: null,
        render: null,
        world: null,
        mode: 'ASSEMBLY', // Initial mode
        canvas: null,
        canvasWidth: 800, // Default, will be updated
        canvasHeight: 600, // Default, will be updated
        playerRobot: { parts: [], constraints: [] },
        enemyRobot: { parts: [], constraints: [] },
        dragging: {
            isDragging: false,
            partType: null,
            ghostElement: null, // Visual representation while dragging
            offsetX: 0, // Offset from cursor to part's top-left
            offsetY: 0,
            originalElement: null, // The inventory icon that was clicked
            isSnapped: false,
            snapTargetBody: null, // The body we are snapping to
            snapTargetPoint: null, // The specific connection point object on the target body
            snapSourcePoint: null, // The specific connection point on the source (dragged) part
            snapConstraint: null // Info for creating the constraint later
        },
        playerControls: {
            forward: false,
            backward: false,
            left: false,
            right: false
        },
        ui: {
            partsGrid: document.querySelector('.parts-grid'),
            jsonTextarea: document.getElementById('json-textarea'),
            saveButton: document.getElementById('save-button'),
            loadButton: document.getElementById('load-button'),
            clearButton: document.getElementById('clear-button'),
            startBattleButton: document.getElementById('start-battle-button'),
            gameCanvas: document.getElementById('game-canvas'),
            gameTitle: document.querySelector('.game-title'),
            // Added for Game Over
            gameOverScreen: document.getElementById('game-over-screen'),
            gameOverWinner: document.getElementById('game-over-winner'),
            backToEditorButton: document.getElementById('back-to-editor-button'),
        },
        // More properties will be added as needed
    };

    // PDS 5.1: Matter.js aliases
    const Engine = Matter.Engine;
    const Render = Matter.Render;
    const Runner = Matter.Runner;
    const World = Matter.World;
    const Bodies = Matter.Bodies;
    const Composite = Matter.Composite;
    const Events = Matter.Events;
    const Constraint = Matter.Constraint;

const PLAYER_TORQUE_AMOUNT = 0.15; // Adjusted torque, may need further tuning
const AI_TORQUE_AMOUNT = 0.08; // Can be tuned
const BASE_DAMAGE_MULTIPLIER = 2; // PDS 3.3
const MIN_COLLISION_SPEED_FOR_DAMAGE = 2; // PDS 3.3

function handlePlayerInput(event, isKeyDown) {
    // Prevent default browser action for arrow keys, etc. if they scroll the page
    if (['arrowup', 'arrowdown', 'arrowleft', 'arrowright', 'w', 'a', 's', 'd'].includes(event.key.toLowerCase())) {
        event.preventDefault();
    }

    switch (event.key.toLowerCase()) {
        case 'w':
        case 'arrowup':
            gameState.playerControls.forward = isKeyDown;
            break;
        case 's':
        case 'arrowdown':
            gameState.playerControls.backward = isKeyDown;
            break;
        case 'a':
        case 'arrowleft':
            gameState.playerControls.left = isKeyDown;
            break;
        case 'd':
        case 'arrowright':
            gameState.playerControls.right = isKeyDown;
            break;
    }
}

function applyPlayerControls() {
    if (gameState.mode !== 'BATTLE' || !gameState.playerRobot || gameState.playerRobot.parts.length === 0) {
        return;
    }

    const { forward, backward, left, right } = gameState.playerControls;
    const mainChassis = gameState.playerRobot.parts.find(p => p.customProps && p.customProps.type === 'chassis' && p.customProps.isPlayerPart);

    gameState.playerRobot.parts.forEach(part => {
        if (part.customProps && part.customProps.type === 'wheel' && part.customProps.isPlayerPart) {
            let baseTorque = 0;
            if (forward) baseTorque += PLAYER_TORQUE_AMOUNT;
            if (backward) baseTorque -= PLAYER_TORQUE_AMOUNT;

            let turnTorque = 0;
            if (mainChassis) {
                const relevantConstraint = gameState.playerRobot.constraints.find(c =>
                    (c.bodyA === mainChassis && c.bodyB === part) ||
                    (c.bodyB === mainChassis && c.bodyA === part)
                );

                if (relevantConstraint) {
                    const pointOnChassis = (relevantConstraint.bodyA === mainChassis) ? relevantConstraint.pointA : relevantConstraint.pointB;
                    // Assuming chassis's local (0,0) is its center.
                    // pointA/B are relative to the body's center.
                    if (pointOnChassis.x < -1) { // Wheel is on the left side of chassis
                        if (left) turnTorque -= PLAYER_TORQUE_AMOUNT * 0.85;
                        if (right) turnTorque += PLAYER_TORQUE_AMOUNT * 0.85;
                    } else if (pointOnChassis.x > 1) { // Wheel is on the right side of chassis
                        if (left) turnTorque += PLAYER_TORQUE_AMOUNT * 0.85;
                        if (right) turnTorque -= PLAYER_TORQUE_AMOUNT * 0.85;
                    } else { // Wheel is central
                         if (left) turnTorque -= PLAYER_TORQUE_AMOUNT * 0.4;
                         if (right) turnTorque += PLAYER_TORQUE_AMOUNT * 0.4;
                    }
                } else { // Wheel not directly constrained to main chassis - simpler spin
                    if (left) turnTorque -= PLAYER_TORQUE_AMOUNT * 0.5;
                    if (right) turnTorque += PLAYER_TORQUE_AMOUNT * 0.5;
                }
            } else { // No main chassis found, apply simpler spin torque
                if (left) turnTorque -= PLAYER_TORQUE_AMOUNT * 0.5;
                if (right) turnTorque += PLAYER_TORQUE_AMOUNT * 0.5;
            }
            Matter.Body.setTorque(part, baseTorque + turnTorque);
        }
    });
}

const SNAP_RADIUS = 30; // PDS 2.2.2: 30px

/**
 * PDS 2.2.2: Checks for snap points and updates ghost position if snapped.
 */
function checkSnapPoints(event) {
    if (!gameState.dragging.isDragging || !gameState.dragging.partType) return;

    const draggedPartDef = componentDefaults[gameState.dragging.partType];
    if (!draggedPartDef || !draggedPartDef.connectionPoints || draggedPartDef.connectionPoints.length === 0) {
        clearSnapState(); // No connection points on the dragged part
        return;
    }

    const mousePos = getMousePosInCanvas(event);
    let bestSnap = null;
    let minDistanceSq = SNAP_RADIUS * SNAP_RADIUS;

    // Iterate over all parts already on the player's robot
    for (const existingPart of gameState.playerRobot.parts) {
        if (!existingPart.customProps || !existingPart.customProps.connectionPoints) continue;

        for (const targetPointDef of existingPart.customProps.connectionPoints) {
            if (targetPointDef.isOccupied) continue; // Skip if already used by another constraint

            // Calculate world position of the target connection point
            const targetPointWorld = Matter.Vector.add(
                existingPart.position,
                Matter.Vector.rotate({ x: targetPointDef.x, y: targetPointDef.y }, existingPart.angle)
            );

            // Iterate over connection points of the part being dragged
            for (const sourcePointDef of draggedPartDef.connectionPoints) {
                // Calculate the hypothetical world position of the source connection point if the part was placed at mousePos
                // This means the sourcePointDef (relative to dragged part's center) should align with targetPointWorld
                // So, dragged part's center would be: targetPointWorld - sourcePointDef (rotated by dragged part's current/default angle)
                // For simplicity in finding a snap, we first check distance from mouse to targetPointWorld.
                // A more accurate check involves iterating rotations or assuming a default orientation for snapping.
                // Let's assume the dragged part snaps with its default orientation (angle 0 for now).

                const distanceSq = Matter.Vector.magnitudeSquared(Matter.Vector.sub(targetPointWorld, mousePos));

                if (distanceSq < minDistanceSq) {
                    minDistanceSq = distanceSq;
                    bestSnap = {
                        targetBody: existingPart,
                        targetPoint: targetPointDef, // The definition {x, y, type}
                        targetPointWorld: targetPointWorld, // Calculated world coords
                        sourcePartType: gameState.dragging.partType,
                        sourcePoint: sourcePointDef, // The definition {x, y, type}
                    };
                }
            }
        }
    }

    if (bestSnap) {
        gameState.dragging.isSnapped = true;
        gameState.dragging.snapTargetBody = bestSnap.targetBody;
        gameState.dragging.snapTargetPoint = bestSnap.targetPoint; // e.g. {x:0, y:-40, type:'top'}
        gameState.dragging.snapSourcePoint = bestSnap.sourcePoint; // e.g. {x:0, y:30, type:'base'}

        // Adjust ghost position:
        // The ghost part's sourcePoint should align with targetPointWorld.
        // The ghost part's center = targetPointWorld - (rotated sourcePoint relative vector)
        // Assuming ghost is not rotated for now for simplicity of ghost vis.
        const ghostPartCenter = Matter.Vector.sub(
            bestSnap.targetPointWorld,
            { x: bestSnap.sourcePoint.x, y: bestSnap.sourcePoint.y } // Assuming ghost angle 0
        );

        // Convert canvas coords back to screen coords for the ghost div
        const canvasRect = gameState.ui.gameCanvas.getBoundingClientRect();
        const screenX = ghostPartCenter.x + canvasRect.left;
        const screenY = ghostPartCenter.y + canvasRect.top;

        // Update ghost element to align its center with ghostPartCenter
        const ghostRect = gameState.dragging.ghostElement.getBoundingClientRect();
        gameState.dragging.ghostElement.style.left = `${screenX - ghostRect.width / 2}px`;
        gameState.dragging.ghostElement.style.top = `${screenY - ghostRect.height / 2}px`;

        // Store info needed for creating constraint
        gameState.dragging.snapConstraint = {
            bodyA: bestSnap.targetBody,
            // bodyB will be the new part
            pointA: { x: bestSnap.targetPoint.x, y: bestSnap.targetPoint.y }, // Relative to bodyA's center
            // pointB will be relative to bodyB's center
            pointB: { x: bestSnap.sourcePoint.x, y: bestSnap.sourcePoint.y }
        };

    } else {
        clearSnapState();
    }
    renderSnapHighlights(); // Visual feedback for snapping
}

function clearSnapState() {
    gameState.dragging.isSnapped = false;
    gameState.dragging.snapTargetBody = null;
    gameState.dragging.snapTargetPoint = null;
    gameState.dragging.snapSourcePoint = null;
    gameState.dragging.snapConstraint = null;
}

/**
 * PDS 2.2.2: Renders snap point highlights on canvas.
 */
function renderSnapHighlights() {
    // This function is called continuously by the game loop or after checkSnapPoints
    // For now, we'll just log, actual rendering on canvas needs integration with Matter.Render or custom drawing.

    // A simple way to do this is to draw directly on the main canvas after Matter.js render.
    // This requires hooking into Events.on(gameState.render, 'afterRender', drawHighlights);
    // For now, we'll rely on ghost position and console logs.

    if (gameState.dragging.isSnapped && gameState.dragging.snapTargetPoint) {
        // console.log(`Snapped to ${gameState.dragging.snapTargetBody.customProps.label} at point type ${gameState.dragging.snapTargetPoint.type}`);
        // Ideally, highlight gameState.dragging.snapTargetPoint on canvas
    }
}

// Add a visual way to see connection points (for debugging and PDS requirement)
// This needs to be called in a render loop (e.g., Matter.Render 'afterRender' event)
function drawConnectionPoints(event) {
    const context = gameState.render.context; // Get the canvas 2D context from Matter.js render

    context.save(); // Save current drawing state
    context.lineWidth = 1;

    for (const part of gameState.playerRobot.parts) {
        if (!part.customProps || !part.customProps.connectionPoints) continue;

        for (const cp of part.customProps.connectionPoints) {
            const worldPos = Matter.Vector.add(part.position, Matter.Vector.rotate({x: cp.x, y: cp.y}, part.angle));

            context.beginPath();
            context.arc(worldPos.x, worldPos.y, 5, 0, 2 * Math.PI); // Draw a small circle for CP

            if (cp.isOccupied) {
                context.fillStyle = 'rgba(255, 0, 0, 0.7)'; // Red if occupied
            } else {
                context.fillStyle = 'rgba(0, 255, 0, 0.7)'; // Green if available (PDS 2.2.2)
            }
            context.fill();

            // Highlight if this is the current snap target
            if (gameState.dragging.isSnapped &&
                gameState.dragging.snapTargetBody === part &&
                gameState.dragging.snapTargetPoint === cp) {
                context.strokeStyle = 'rgba(0, 255, 0, 1)'; // Brighter green for active snap
                context.lineWidth = 2;
                context.stroke();
            }
        }
    }
    context.restore(); // Restore drawing state
}

    /**
     * PDS 5.2: Main entry point, called on DOMContentLoaded
     */
    function main() {
        console.log("Robot Battle Initializing...");
        initEngine();
        initControls(); // Basic controls init
        initComponents(); // Will be implemented in a later step
        // initAudio(); // Will be implemented in a later step

        console.log("Robot Battle Initialized and Running.");
    }

    /**
     * PDS 3.1: Initialize Matter.js Engine, Renderer, and World
     */
    function initEngine() {
        gameState.engine = Engine.create();
        gameState.world = gameState.engine.world;

        // Configure canvas dimensions based on container
        const canvasContainer = gameState.ui.gameCanvas.parentElement;
        gameState.canvasWidth = canvasContainer.clientWidth * 0.98; // Account for padding/border
        gameState.canvasHeight = canvasContainer.clientHeight * 0.98; // Account for padding/border


        gameState.ui.gameCanvas.width = gameState.canvasWidth;
        gameState.ui.gameCanvas.height = gameState.canvasHeight;


        gameState.render = Render.create({
            canvas: gameState.ui.gameCanvas,
            engine: gameState.engine,
            options: {
                width: gameState.canvasWidth,
                height: gameState.canvasHeight,
                wireframes: false, // Show solid shapes
                background: '#2c2c2e' // Canvas background from CSS
            }
        });

        // PDS 3.1: Gravity
        gameState.engine.world.gravity.y = 1.0;

        // PDS 3.1: Environment - Arena Walls
        const wallThickness = 50;
        const wallOptions = {
            isStatic: true,
            restitution: 0.1, // Low restitution
            friction: 0.9,    // PDS: High friction
            render: { fillStyle: '#4a4a4a' } // Dark grey walls
        };

        World.add(gameState.world, [
            // Ground
            Bodies.rectangle(gameState.canvasWidth / 2, gameState.canvasHeight + wallThickness / 2, gameState.canvasWidth, wallThickness, wallOptions),
            // Ceiling
            Bodies.rectangle(gameState.canvasWidth / 2, -wallThickness / 2, gameState.canvasWidth, wallThickness, wallOptions),
            // Left Wall
            Bodies.rectangle(-wallThickness / 2, gameState.canvasHeight / 2, wallThickness, gameState.canvasHeight, wallOptions),
            // Right Wall
            Bodies.rectangle(gameState.canvasWidth + wallThickness / 2, gameState.canvasHeight / 2, wallThickness, gameState.canvasHeight, wallOptions)
        ]);

        Render.run(gameState.render);
        Events.on(gameState.render, 'afterRender', drawConnectionPoints); // PDS 2.2.2 Hook for drawing overlays
        const runner = Runner.create();
        Runner.run(runner, gameState.engine);

        // PDS 5.2: Game Loop (placeholder, will be expanded)
        Events.on(gameState.engine, 'beforeUpdate', (event) => {
            if (gameState.mode === 'BATTLE') {
                applyPlayerControls();
                applyAIControls();
                checkAndProcessDestruction(); // <<< ADD THIS CALL
            }
            // Other game loop logic can go here if needed
        });

        // PDS 3.3: Collision Handling
        Events.on(gameState.engine, 'collisionStart', (event) => {
            handleCollisions(event.pairs); // Pass all pairs to the handler
        });

        console.log("Matter.js Engine and Renderer Initialized.");
        console.log(`Canvas dimensions: ${gameState.canvasWidth}x${gameState.canvasHeight}`);
    }

    /**
     * PDS 5.2: Initialize basic event listeners
     */
    function initControls() {
        // Event listeners for UI buttons will be added in detail later
        // gameState.ui.saveButton.addEventListener('click', handleSaveDesign);
        // gameState.ui.loadButton.addEventListener('click', handleLoadDesign);
        // gameState.ui.clearButton.addEventListener('click', handleClearCanvas);
        if (gameState.ui.startBattleButton) { // Check if button exists
            gameState.ui.startBattleButton.addEventListener('click', startBattle);
        } else {
            console.error("Start Battle Button not found in gameState.ui");
        }
        document.addEventListener('keydown', (event) => handlePlayerInput(event, true));
        document.addEventListener('keyup', (event) => handlePlayerInput(event, false));
        console.log("Keyboard listeners for player controls added.");

        if (gameState.ui.backToEditorButton) {
            gameState.ui.backToEditorButton.addEventListener('click', () => {
                resetToEditor();
            });
        } else {
            console.error("Back to Editor Button not found in gameState.ui");
        }
        console.log("Basic Controls Initialized.");
    }

    /**
     * PDS 5.2: Populates the parts inventory UI
     */
    function initComponents() {
        gameState.ui.partsGrid.innerHTML = ''; // Clear existing parts, if any

        for (const partType in componentDefaults) {
            const partDef = componentDefaults[partType];
            const partElement = document.createElement('div');
            partElement.classList.add('part-icon');
            partElement.setAttribute('title', `Drag to add ${partDef.label}`);
            partElement.setAttribute('data-part-type', partType);

            // Simple text label for now, could be an SVG icon later
            const label = document.createElement('span');
            label.textContent = partDef.label.charAt(0).toUpperCase() + partDef.label.slice(1);
            partElement.appendChild(label);

            // Add styling for the part icon based on its definition (optional, but good for UI)
            // For example, using the render fillStyle for background
            partElement.style.backgroundColor = partDef.render.fillStyle || '#444';
            partElement.style.borderColor = partDef.render.strokeStyle || '#333';
            partElement.style.color = '#FFF'; // Ensure text is visible

            // Placeholder for drag start event
            partElement.addEventListener('mousedown', (e) => {
                initiateDrag(e, partType); // <<< USE THIS
            });

            gameState.ui.partsGrid.appendChild(partElement);
        }
        console.log("Parts Inventory UI Initialized.");
    }

    // Helper function to get mouse position relative to canvas
    function getMousePosInCanvas(event) {
        const rect = gameState.ui.gameCanvas.getBoundingClientRect();
        return {
            x: event.clientX - rect.left,
            y: event.clientY - rect.top
        };
    }

    /**
     * Initiates dragging of a part from the inventory.
     * PDS 2.2.1
     */
    function initiateDrag(event, partType) {
        console.log('[DRAG_INIT] Attempting to initiate drag for partType:', partType, 'Event:', event);
        if (gameState.dragging.isDragging) {
            console.warn('[DRAG_INIT] Drag already in progress. Ignoring.');
            return;
        }

        const partDef = componentDefaults[partType];
        if (!partDef) {
            console.error('[DRAG_INIT] Unknown partType:', partType);
            return;
        }
        console.log('[DRAG_INIT] Part definition found:', partDef);

        gameState.dragging.isDragging = true;
        gameState.dragging.partType = partType;
        gameState.dragging.originalElement = event.currentTarget;

        gameState.dragging.ghostElement = gameState.dragging.originalElement.cloneNode(true);
        gameState.dragging.ghostElement.classList.add('part-ghost');
        gameState.dragging.ghostElement.style.position = 'absolute';
        gameState.dragging.ghostElement.style.zIndex = '1000';
        gameState.dragging.ghostElement.style.opacity = '0.7'; // Semi-transparent
        gameState.dragging.ghostElement.style.pointerEvents = 'none'; // Don't interfere with mouse events

        // Calculate offset from cursor to top-left of the clicked inventory icon
        const originalRect = gameState.dragging.originalElement.getBoundingClientRect();
        gameState.dragging.offsetX = event.clientX - originalRect.left;
        gameState.dragging.offsetY = event.clientY - originalRect.top;

        document.body.appendChild(gameState.dragging.ghostElement);
        console.log('[DRAG_INIT] Ghost element created and appended:', gameState.dragging.ghostElement);
        moveGhostElement(event);

        document.addEventListener('mousemove', handleDragMove);
        document.addEventListener('mouseup', handleDragEnd);
        console.log('[DRAG_INIT] Global mousemove and mouseup listeners added.');
        console.log(`[DRAG_INIT] Dragging ${partType} started. Initial ghost position: left=${gameState.dragging.ghostElement.style.left}, top=${gameState.dragging.ghostElement.style.top}`);
    }

    /**
     * Moves the ghost element with the cursor.
     * PDS 2.2.1
     */
    function moveGhostElement(event) {
        if (!gameState.dragging.isDragging || !gameState.dragging.ghostElement) return;

        // Position ghost element based on cursor, adjusted by the initial offset
        gameState.dragging.ghostElement.style.left = `${event.clientX - gameState.dragging.offsetX}px`;
        gameState.dragging.ghostElement.style.top = `${event.clientY - gameState.dragging.offsetY}px`;
        // console.log('[DRAG_MOVE_GHOST] Ghost moved to left:', gameState.dragging.ghostElement.style.left, 'top:', gameState.dragging.ghostElement.style.top); // Can be too verbose
    }

    /**
     * Handles mouse movement during dragging.
     */
    function handleDragMove(event) {
        // console.log('[DRAG_MOVE] handleDragMove called. Snapped:', gameState.dragging.isSnapped); // Can be too verbose
        if (!gameState.dragging.isDragging) return;

        if (gameState.dragging.isSnapped) {
            // console.log('[DRAG_MOVE] Is snapped, re-checking snap points.'); // Can be too verbose
            checkSnapPoints(event);
        } else {
            moveGhostElement(event);
        }
        checkSnapPoints(event);
    }


    // Modify handleDragEnd to use snap information
    function handleDragEnd(event) {
        console.log('[DRAG_END] handleDragEnd called. Event:', event);
        if (!gameState.dragging.isDragging) {
            console.warn('[DRAG_END] Not currently dragging. Exiting.');
            return;
        }

        let placedPart = null;
        const mousePos = getMousePosInCanvas(event);
        const canvasRect = gameState.ui.gameCanvas.getBoundingClientRect();
        const isOverCanvas = event.clientX >= canvasRect.left && event.clientX <= canvasRect.right &&
                             event.clientY >= canvasRect.top && event.clientY <= canvasRect.bottom;

        console.log(`[DRAG_END] Mouse released at screen (clientX:${event.clientX}, clientY:${event.clientY}), canvas local (x:${mousePos.x}, y:${mousePos.y}). IsOverCanvas: ${isOverCanvas}. IsSnapped: ${gameState.dragging.isSnapped}`);

        if (gameState.dragging.isSnapped && gameState.dragging.snapTargetBody && gameState.dragging.snapSourcePoint) {
            const targetBody = gameState.dragging.snapTargetBody;
            const targetPointDef = gameState.dragging.snapTargetPoint;
            const sourcePointDef = gameState.dragging.snapSourcePoint;
            const newPartAngle = targetBody.angle;
            const targetPointWorld = Matter.Vector.add(
                targetBody.position,
                Matter.Vector.rotate({ x: targetPointDef.x, y: targetPointDef.y }, targetBody.angle)
            );
            const newPartCenter = Matter.Vector.sub(
                targetPointWorld,
                Matter.Vector.rotate({ x: sourcePointDef.x, y: sourcePointDef.y }, newPartAngle)
            );
            console.log(`[DRAG_END_SNAP] Part ${gameState.dragging.partType} snapped. Target: ${targetBody.customProps.label}. Calculated New Part Center: (${newPartCenter.x.toFixed(0)}, ${newPartCenter.y.toFixed(0)}), Angle: ${newPartAngle.toFixed(2)}`);
            placedPart = createPartOnCanvas(gameState.dragging.partType, newPartCenter.x, newPartCenter.y, newPartAngle, gameState.dragging.snapConstraint);

            if (placedPart && gameState.dragging.snapConstraint) {
                const constraintDef = gameState.dragging.snapConstraint;
                const constraint = Constraint.create({
                    bodyA: constraintDef.bodyA,
                    bodyB: placedPart,
                    pointA: constraintDef.pointA,
                    pointB: constraintDef.pointB,
                    stiffness: 0.9,
                    length: 0,
                    render: {
                        strokeStyle: '#66B2D9',
                        lineWidth: 2
                    }
                });
                World.add(gameState.world, constraint);
                gameState.playerRobot.constraints.push(constraint);
                console.log('[DRAG_END_SNAP] Constraint created successfully.');
                targetPointDef.isOccupied = true;
                const placedPartSourcePoint = placedPart.customProps.connectionPoints.find(
                    p => p.x === sourcePointDef.x && p.y === sourcePointDef.y && p.type === sourcePointDef.type
                );
                if (placedPartSourcePoint) placedPartSourcePoint.isOccupied = true;
            }
            if (placedPart) {
                console.log('[DRAG_END_SNAP_RENDER_CHECK] Renderer: wireframes=', gameState.render.options.wireframes, 'Canvas offsetParent:', gameState.render.canvas.offsetParent ? 'Exists' : 'null', 'Canvas W/H:', gameState.render.canvas.width, gameState.render.canvas.height);
            }
        } else if (isOverCanvas) {
            console.log(`[DRAG_END_REGULAR] Part ${gameState.dragging.partType} dropped at canvas local (x:${mousePos.x}, y:${mousePos.y})`);
            placedPart = createPartOnCanvas(gameState.dragging.partType, mousePos.x, mousePos.y);
            if (placedPart) {
                console.log('[DRAG_END_REGULAR_RENDER_CHECK] Renderer: wireframes=', gameState.render.options.wireframes, 'Canvas offsetParent:', gameState.render.canvas.offsetParent ? 'Exists' : 'null', 'Canvas W/H:', gameState.render.canvas.width, gameState.render.canvas.height);
            }
        } else {
            console.log("[DRAG_END] Part drag cancelled (released outside canvas).");
        }

        if (gameState.dragging.ghostElement) gameState.dragging.ghostElement.remove();
        clearSnapState();
        gameState.dragging.isDragging = false;
        gameState.dragging.partType = null;
        gameState.dragging.ghostElement = null;
        gameState.dragging.originalElement = null;
        document.removeEventListener('mousemove', handleDragMove);
        document.removeEventListener('mouseup', handleDragEnd);
        console.log("[DRAG_END] Dragging operation fully cleaned up and ended.");
    }

    /**
     * Creates a physical Matter.js part on the canvas.
     * PDS 2.2.1
     */
    function createPartOnCanvas(partType, x, y, angle = 0, snappedTo = null) { // Added snappedTo parameter
        console.log(`[CREATE_PART] Called with: partType=${partType}, x=${x}, y=${y}, angle=${angle}, snappedTo:`, snappedTo);
        const partDef = componentDefaults[partType];
        if (!partDef) {
            console.error(`[CREATE_PART] Unknown part type: ${partType}`);
            return null;
        }
        console.log('[CREATE_PART] Part definition:', partDef);

        let body;
        const commonOptions = {
            angle: angle,
            mass: partDef.mass, // PDS 3.2
            // PDS 3.1: Player robot parts belong to collisionFilter.group = -1
            collisionFilter: { group: -1 },
            render: { ...partDef.render } // Clone render options
        };

        // Attach custom properties directly to the body
        const customProps = {
            label: partDef.label,
            health: partDef.health, // PDS 3.2
            type: partType, // e.g., 'chassis', 'wheel'
            // Deep clone and ensure isOccupied is initialized
            connectionPoints: JSON.parse(JSON.stringify(partDef.connectionPoints || [])).map(cp => ({...cp, isOccupied: false})),
            id: Matter.Common.nextId(), // Unique ID for this part instance
            isPlayerPart: true // Flag for easy identification
        };

    // Ensure this part is not commented out or failing silently
    if (partType === 'chassis') { body = Bodies.rectangle(x, y, partDef.width, partDef.height, commonOptions); }
    else if (partType === 'wheel') { body = Bodies.circle(x, y, partDef.radius, { ...commonOptions, ...partDef.matterJsOptions }); }
    else if (partType === 'spike') { body = Bodies.fromVertices(x, y, [partDef.vertices], commonOptions); }
    else { console.error(`[CREATE_PART] Critical: Body type specific creation logic failed for ${partType}`); return null; }


    if (!body) {
        console.error(`[CREATE_PART] CRITICAL ERROR: Body was not created for partType ${partType}. Check body creation logic.`);
            return null;
        }
        body.customProps = customProps;

    console.log('[CREATE_PART] Body to be added to world:', {
        matterJsBodyId: body.id,
        customPartId: customProps.id,
        partType: customProps.type,
        targetPosition: { x: x, y: y }, // Log target position
        actualPosition: { ...body.position }, // Log actual initial position from Matter
        angle: body.angle,
        renderOptions: { ...body.render },
        bounds: { min: { ...body.bounds.min }, max: { ...body.bounds.max } }
    });

    const bodiesBeforeAdd = gameState.world.bodies.length;
    console.log(`[CREATE_PART] World bodies count BEFORE add: ${bodiesBeforeAdd}`);

        World.add(gameState.world, body);

    const bodiesAfterAdd = gameState.world.bodies.length;
    console.log(`[CREATE_PART] World bodies count AFTER add: ${bodiesAfterAdd}`);

    if (bodiesAfterAdd > bodiesBeforeAdd) {
        const addedBodyInWorld = gameState.world.bodies.find(b => b.id === body.id);
        if (addedBodyInWorld) {
            console.log('[CREATE_PART] Body successfully found in world.bodies by ID. Details:', {
                 id: addedBodyInWorld.id,
                 pos: { ...addedBodyInWorld.position },
                 angle: addedBodyInWorld.angle,
                 label: addedBodyInWorld.label, // Matter's internal label
                 renderVisible: addedBodyInWorld.render.visible,
                 fill: addedBodyInWorld.render.fillStyle,
                 stroke: addedBodyInWorld.render.strokeStyle
            });
        } else {
            console.error('[CREATE_PART] CRITICAL ERROR: Body added to world, but NOT found by ID in world.bodies immediately after. This indicates a major issue!');
        }
    } else {
        console.error('[CREATE_PART] CRITICAL ERROR: World.add() did not increase bodies count. Part NOT added to world.');
    }

    gameState.playerRobot.parts.push(body);
    console.log(`[CREATE_PART] ${customProps.label} (ID: ${customProps.id}) added to playerRobot.parts. Final log from createPartOnCanvas.`);
        return body;
    }

    // Start the game once the DOM is fully loaded
    document.addEventListener('DOMContentLoaded', main);

// Placeholder for handleGameOver - will be fully implemented in the next step
function handleGameOver(winner) {
    gameState.mode = 'GAMEOVER'; // Halt battle logic

    // Update text content
    if (gameState.ui.gameOverWinner) {
        switch (winner.toLowerCase()) {
            case 'player':
                gameState.ui.gameOverWinner.textContent = "Player Wins!";
                break;
            case 'enemy':
                gameState.ui.gameOverWinner.textContent = "Enemy Wins!";
                break;
            case 'draw (simultaneous)':
                 gameState.ui.gameOverWinner.textContent = "It's a Draw!";
                 break;
            default:
                gameState.ui.gameOverWinner.textContent = "Match Ended.";
        }
    }

    // Display the game over screen
    if (gameState.ui.gameOverScreen) {
        gameState.ui.gameOverScreen.style.display = 'flex';
    }

    console.log(`Game Over! Winner: ${winner}.`);
}

function checkAndProcessDestruction() {
    if (gameState.mode !== 'BATTLE') return;

    let playerChassisDestroyed = false;
    let enemyChassisDestroyed = false;

    // Helper function to process destruction for a single robot's parts list
    const processRobotParts = (robotParts, robotConstraints, owner) => {
        for (let i = robotParts.length - 1; i >= 0; i--) {
            const part = robotParts[i];
            if (part.customProps && part.customProps.health <= 0) {
                console.log(`${owner} part ID ${part.customProps.id} (${part.customProps.label}) is being removed due to zero health.`);

                // Cosmetic Explosion placeholder
                console.log(`Part ID ${part.customProps.id} visual explosion!`);

                // Check for chassis destruction
                if (part.customProps.label === 'chassis') {
                    if (owner === 'Player') playerChassisDestroyed = true;
                    if (owner === 'Enemy') enemyChassisDestroyed = true;
                }

                // Remove constraints attached to this part
                for (let j = robotConstraints.length - 1; j >= 0; j--) {
                    const constraint = robotConstraints[j];
                    if (constraint.bodyA === part || constraint.bodyB === part) {
                        World.remove(gameState.world, constraint);
                        robotConstraints.splice(j, 1);
                        console.log(`Constraint ${constraint.id} attached to part ${part.customProps.id} removed.`);
                    }
                }

                // Remove the part from Matter world
                World.remove(gameState.world, part);
                // Remove the part from the game state array
                robotParts.splice(i, 1);
            }
        }
    };

    // Process player robot
    processRobotParts(gameState.playerRobot.parts, gameState.playerRobot.constraints, 'Player');
    // Process enemy robot
    processRobotParts(gameState.enemyRobot.parts, gameState.enemyRobot.constraints, 'Enemy');

    // Handle Game Over condition (implementation of handleGameOver is next step)
    if (playerChassisDestroyed && enemyChassisDestroyed) {
        console.log("Simultaneous chassis destruction! It's a draw?");
        // handleGameOver("Draw"); // Or based on which was detected first if possible
        if (typeof handleGameOver === 'function') handleGameOver("Draw (Simultaneous)"); // Placeholder
    } else if (playerChassisDestroyed) {
        console.log("Player chassis destroyed! Enemy wins!");
        if (typeof handleGameOver === 'function') handleGameOver("Enemy"); // Placeholder
    } else if (enemyChassisDestroyed) {
        console.log("Enemy chassis destroyed! Player wins!");
        if (typeof handleGameOver === 'function') handleGameOver("Player"); // Placeholder
    }
}

// Define this function within the IIFE:
function handleCollisions(pairs) {
    if (gameState.mode !== 'BATTLE') return;

    pairs.forEach(pair => {
        const { bodyA, bodyB, collision } = pair;

        // Check if collision speed is sufficient to cause damage
        if (collision.speed < MIN_COLLISION_SPEED_FOR_DAMAGE) {
            return;
        }

        // Ensure parts have customProps (meaning they are game parts we track)
        if (!bodyA.customProps || !bodyB.customProps) {
            return;
        }

        // Ensure parts are from different robots (player vs enemy)
        // And neither is static (like a wall, though walls shouldn't have customProps this way)
        const isBodyAPlayer = bodyA.customProps.isPlayerPart === true && bodyA.customProps.isEnemyPart !== true;
        const isBodyBPlayer = bodyB.customProps.isPlayerPart === true && bodyB.customProps.isEnemyPart !== true;
        const isBodyAEnemy = bodyA.customProps.isEnemyPart === true && bodyA.customProps.isPlayerPart !== true;
        const isBodyBEnemy = bodyB.customProps.isEnemyPart === true && bodyB.customProps.isPlayerPart !== true;

        // Only proceed if one is player and the other is enemy
        if (!((isBodyAPlayer && isBodyBEnemy) || (isBodyAEnemy && isBodyBPlayer))) {
            return;
        }

        // Damage Calculation
        const damage = collision.speed * BASE_DAMAGE_MULTIPLIER;

        // Apply damage to both parts involved in the collision
        // Part A
        if (bodyA.customProps.health > 0) { // Check if not already destroyed
            bodyA.customProps.health -= damage;
            console.log(`Part ID ${bodyA.customProps.id} (${bodyA.customProps.label}, ${isBodyAPlayer ? 'Player' : 'Enemy'}) took ${damage.toFixed(1)} damage. Health: ${bodyA.customProps.health.toFixed(1)}`);
            if (bodyA.customProps.health <= 0) {
                bodyA.customProps.health = 0; // Cap at 0
                // Destruction will be handled in the next step (partDestruction)
                console.log(`Part ID ${bodyA.customProps.id} DESTROYED!`);
            }
        }

        // Part B
        if (bodyB.customProps.health > 0) { // Check if not already destroyed
            bodyB.customProps.health -= damage;
            console.log(`Part ID ${bodyB.customProps.id} (${bodyB.customProps.label}, ${isBodyBPlayer ? 'Player' : 'Enemy'}) took ${damage.toFixed(1)} damage. Health: ${bodyB.customProps.health.toFixed(1)}`);
            if (bodyB.customProps.health <= 0) {
                bodyB.customProps.health = 0; // Cap at 0
                console.log(`Part ID ${bodyB.customProps.id} DESTROYED!`);
            }
        }
    });
}

function applyAIControls() {
    if (gameState.mode !== 'BATTLE' || !gameState.enemyRobot || gameState.enemyRobot.parts.length === 0 || !gameState.playerRobot || gameState.playerRobot.parts.length === 0) {
        return;
    }

    const playerChassis = gameState.playerRobot.parts.find(p => p.customProps && p.customProps.type === 'chassis' && p.customProps.isPlayerPart);
    const enemyChassis = gameState.enemyRobot.parts.find(p => p.customProps && p.customProps.type === 'chassis' && p.customProps.isEnemyPart);

    if (!playerChassis || !enemyChassis) {
        // One of the robots (or their chassis) is missing, AI can't operate
        // Stop all enemy wheel torque if a chassis is missing
        gameState.enemyRobot.parts.forEach(part => {
            if (part.customProps && part.customProps.type === 'wheel' && part.customProps.isEnemyPart) {
                 Matter.Body.setTorque(part, 0);
            }
        });
        return;
    }

    // Calculate vector from enemy to player
    const vectorToPlayer = Matter.Vector.sub(playerChassis.position, enemyChassis.position);

    // Calculate angle to player relative to world's x-axis
    // Correct: Math.atan2(vectorToPlayer.y, vectorToPlayer.x)
    const desiredAngle = Math.atan2(vectorToPlayer.y, vectorToPlayer.x);

    // Current enemy chassis angle (0 is along positive x-axis)
    const currentAngle = enemyChassis.angle;

    // Calculate angle difference. Normalize angles to be comparable.
    let angleDifference = desiredAngle - currentAngle;

    // Normalize angle difference to the range -PI to PI
    while (angleDifference > Math.PI) angleDifference -= 2 * Math.PI;
    while (angleDifference < -Math.PI) angleDifference += 2 * Math.PI;

    // PDS 3.4 AI Logic: SEEKING (implicit), ALIGNING, ADVANCING
    let alignTorque = 0;
    let advanceTorque = 0;

    if (Math.abs(angleDifference) > 0.1) { // ALIGNING state: 0.1 radians is approx 5.7 degrees
        // Apply torque to turn. Positive angleDifference means enemy needs to turn counter-clockwise.
        // Negative angleDifference means enemy needs to turn clockwise.
        alignTorque = (angleDifference > 0) ? -AI_TORQUE_AMOUNT : AI_TORQUE_AMOUNT;
    } else { // ADVANCING state
        advanceTorque = AI_TORQUE_AMOUNT; // Apply forward torque
    }

    gameState.enemyRobot.parts.forEach(part => {
        if (part.customProps && part.customProps.type === 'wheel' && part.customProps.isEnemyPart) {
            let wheelTorque = 0;
            wheelTorque += advanceTorque; // All wheels get forward/backward torque equally for advancing

            // Apply differential torque for alignment
            const relevantConstraint = gameState.enemyRobot.constraints.find(c =>
                (c.bodyA === enemyChassis && c.bodyB === part) ||
                (c.bodyB === enemyChassis && c.bodyA === part)
            );

            if (relevantConstraint) {
                const pointOnChassis = (relevantConstraint.bodyA === enemyChassis) ? relevantConstraint.pointA : relevantConstraint.pointB;
                if (pointOnChassis.x < -1) { // Wheel on left side of enemy chassis
                    wheelTorque += alignTorque * 0.9;
                } else if (pointOnChassis.x > 1) { // Wheel on right side of enemy chassis
                    wheelTorque -= alignTorque * 0.9;
                } else { // Central wheel
                     wheelTorque += alignTorque * 0.4;
                }
            } else { // Wheel not constrained to chassis, just add base align torque (spin)
                 wheelTorque += alignTorque;
            }
            Matter.Body.setTorque(part, wheelTorque);
        }
    });
}

// Helper function to create a predefined enemy robot
function createEnemyRobot(x, y) {
    const enemyParts = [];
    const enemyConstraints = [];
    const commonEnemyOptions = {
        collisionFilter: { group: -2 }, // PDS 3.1
        isEnemyPart: true // Custom flag
    };

    // Enemy Chassis
    const chassisDef = componentDefaults.chassis;
    const enemyChassis = Bodies.rectangle(x, y, chassisDef.width, chassisDef.height, {
        ...commonEnemyOptions,
        mass: chassisDef.mass,
        render: { ...chassisDef.render, fillStyle: '#B71C1C' } // Dark red chassis
    });
    enemyChassis.customProps = {
        label: 'chassis',
        health: chassisDef.health,
        type: 'chassis',
        connectionPoints: JSON.parse(JSON.stringify(chassisDef.connectionPoints)).map(cp => ({...cp, isOccupied: false})),
        id: Matter.Common.nextId(),
        isEnemyPart: true
    };
    enemyParts.push(enemyChassis);

    // Enemy Wheel 1 (Left)
    const wheelDef = componentDefaults.wheel;
    const wheelOffsetXB = chassisDef.width / 2 - wheelDef.radius / 2 - 5; // Position relative to chassis center
    const wheelOffsetYB = chassisDef.height / 2 - wheelDef.radius / 2;

    const enemyWheel1 = Bodies.circle(x - wheelOffsetXB, y + wheelOffsetYB, wheelDef.radius, {
        ...commonEnemyOptions,
        mass: wheelDef.mass,
        friction: wheelDef.matterJsOptions.friction,
        render: { ...wheelDef.render, fillStyle: '#757575' } // Darker grey wheels
    });
    enemyWheel1.customProps = { label: 'wheel', health: wheelDef.health, type: 'wheel', id: Matter.Common.nextId(), isEnemyPart: true };
    enemyParts.push(enemyWheel1);
    const constraintW1 = Constraint.create({
        bodyA: enemyChassis, bodyB: enemyWheel1,
        pointA: { x: -wheelOffsetXB, y: wheelOffsetYB }, // Relative to chassis center
        pointB: { x: 0, y: 0 }, // Relative to wheel center
        stiffness: 0.7, length: 0, render: { strokeStyle: '#B71C1C' }
    });
    enemyConstraints.push(constraintW1);

    // Enemy Wheel 2 (Right)
    const enemyWheel2 = Bodies.circle(x + wheelOffsetXB, y + wheelOffsetYB, wheelDef.radius, {
        ...commonEnemyOptions,
        mass: wheelDef.mass,
        friction: wheelDef.matterJsOptions.friction,
        render: { ...wheelDef.render, fillStyle: '#757575' }
    });
    enemyWheel2.customProps = { label: 'wheel', health: wheelDef.health, type: 'wheel', id: Matter.Common.nextId(), isEnemyPart: true };
    enemyParts.push(enemyWheel2);
    const constraintW2 = Constraint.create({
        bodyA: enemyChassis, bodyB: enemyWheel2,
        pointA: { x: wheelOffsetXB, y: wheelOffsetYB }, // Relative to chassis center
        pointB: { x: 0, y: 0 }, // Relative to wheel center
        stiffness: 0.7, length: 0, render: { strokeStyle: '#B71C1C' }
    });
    enemyConstraints.push(constraintW2);

    // Enemy Spike
    const spikeDef = componentDefaults.spike;
    const spikeOffsetY = -chassisDef.height / 2 - spikeDef.height / 2; // Mount on top front
    const enemySpike = Bodies.fromVertices(x, y + spikeOffsetY, [spikeDef.vertices], {
        ...commonEnemyOptions,
        mass: spikeDef.mass,
        render: { ...spikeDef.render, fillStyle: '#FF6F00' } // Orange spike
    });
    enemySpike.customProps = { label: 'spike', health: spikeDef.health, type: 'spike', id: Matter.Common.nextId(), isEnemyPart: true };
    enemyParts.push(enemySpike);
    const constraintSpike = Constraint.create({
        bodyA: enemyChassis, bodyB: enemySpike,
        pointA: { x: 0, y: -chassisDef.height / 2 }, // Connect to top of chassis
        pointB: { x: 0, y: spikeDef.height / 2 },   // Connect to base of spike (adjust if vertices origin is different)
        stiffness: 0.9, length: 0, render: { strokeStyle: '#B71C1C' }
    });
    enemyConstraints.push(constraintSpike);

    // Add all enemy parts and constraints to the world
    World.add(gameState.world, [...enemyParts, ...enemyConstraints]);
    gameState.enemyRobot.parts = enemyParts;
    gameState.enemyRobot.constraints = enemyConstraints;
    console.log(`Enemy robot created with ${enemyParts.length} parts and ${enemyConstraints.length} constraints.`);
}


// Modify the existing spawnRobots function
function spawnRobots() {
    console.log("Spawning robots for battle...");

    // 1. Player Robot Preparation
    // Ensure player parts have correct collision group (should be set at creation, but double check)
    gameState.playerRobot.parts.forEach(part => {
        part.collisionFilter.group = -1;
        // Wake up parts if they were sleeping
        Matter.Sleeping.set(part, false);
    });

    // Calculate current bounding box of player's robot
    if (gameState.playerRobot.parts.length > 0) {
        // Correct way to get bounds for multiple bodies: pass the bodies to Bounds.create
        // Matter.js will then iterate through their vertices internally.
        const playerBounds = Matter.Bounds.create(gameState.playerRobot.parts); // Pass array of bodies

        if (playerBounds && typeof playerBounds.min.x === 'number' && typeof playerBounds.min.y === 'number') { // Check if bounds are valid numbers
            const playerRobotWidth = playerBounds.max.x - playerBounds.min.x;
            // const playerRobotHeight = playerBounds.max.y - playerBounds.min.y;

            const targetPlayerX = playerRobotWidth / 2 + 50; // 50px padding from left wall
            const currentRobotCenterX = (playerBounds.min.x + playerBounds.max.x) / 2;

            const currentRobotCenterY = (playerBounds.min.y + playerBounds.max.y) / 2;
            const actualTargetPlayerY = gameState.canvasHeight / 2; // Vertically center the robot

            const dxPlayer = targetPlayerX - currentRobotCenterX;
            const dyPlayer = actualTargetPlayerY - currentRobotCenterY;

            gameState.playerRobot.parts.forEach(part => {
                Matter.Body.translate(part, { x: dxPlayer, y: dyPlayer });
                Matter.Body.setVelocity(part, { x: 0, y: 0 });
                Matter.Body.setAngularVelocity(part, 0);
            });
            console.log(`Player robot repositioned by dx:${dxPlayer.toFixed(0)}, dy:${dyPlayer.toFixed(0)} to target center X:${targetPlayerX.toFixed(0)}, Y:${actualTargetPlayerY.toFixed(0)}`);
        } else {
            console.warn("Could not calculate valid bounds for player robot repositioning. Parts might not be positioned correctly for battle. Bounds:", playerBounds);
        }
    }


    // 2. Enemy Robot Creation and Spawning
    const enemySpawnX = gameState.canvasWidth - 150; // Spawn on the right side
    const enemySpawnY = gameState.canvasHeight / 2;
    createEnemyRobot(enemySpawnX, enemySpawnY);

    console.log("Robots spawned.");
}

function clearBattlefield() {
    console.log("Placeholder: clearBattlefield() called. Clearing logic to be implemented.");
    // This will involve removing enemy robot parts, projectiles etc. from Matter.world
    // and resetting player robot if necessary, or reloading from saved state.
    // For now, we just clear the visual robot arrays.
    if (gameState.world && gameState.enemyRobot && gameState.enemyRobot.parts) {
        gameState.enemyRobot.parts.forEach(part => World.remove(gameState.world, part));
    }
    gameState.enemyRobot.parts = [];

    if (gameState.world && gameState.enemyRobot && gameState.enemyRobot.constraints) {
        gameState.enemyRobot.constraints.forEach(constraint => World.remove(gameState.world, constraint));
    }
    gameState.enemyRobot.constraints = [];

    console.log("Enemy robot parts and constraints cleared from world and gameState.");
}

function startBattle() {
    if (gameState.playerRobot.parts.length === 0) {
        alert("Please build a robot before starting a battle!");
        return;
    }
    // Check if there's at least one chassis for the player
    const playerHasChassis = gameState.playerRobot.parts.some(part => part.customProps && part.customProps.label === 'chassis');
    if (!playerHasChassis) {
        alert("Your robot must have at least one chassis to start a battle!");
        return;
    }

    gameState.mode = 'BATTLE';
    gameState.ui.gameTitle.textContent = "Battle Mode";

    // Hide Assembly UI elements
    const partsInventory = document.querySelector('.parts-inventory');
    if (partsInventory) partsInventory.style.display = 'none';

    const designManagement = document.querySelector('.design-management');
    if (designManagement) designManagement.style.display = 'none';

    if (gameState.ui.startBattleButton) gameState.ui.startBattleButton.style.display = 'none';

    // Placeholder for showing Battle UI elements
    console.log("Battle UI (e.g., controls) should be shown here.");
    // Example: document.getElementById('battle-controls-ui').style.display = 'block';

    console.log("Switched to BATTLE mode.");
    spawnRobots(); // Call to the (soon to be implemented) robot spawning function
}

function resetToEditor() {
    gameState.mode = 'ASSEMBLY';
    gameState.ui.gameTitle.textContent = "Robot Assembly";

    // Show Assembly UI elements
    const partsInventory = document.querySelector('.parts-inventory');
    if (partsInventory) partsInventory.style.display = ''; // Reverts to CSS default

    const designManagement = document.querySelector('.design-management');
    if (designManagement) designManagement.style.display = ''; // Reverts to CSS default

    if (gameState.ui.startBattleButton) gameState.ui.startBattleButton.style.display = ''; // Reverts to CSS default

    // Placeholder for hiding Battle UI elements
    console.log("Assembly UI (e.g., part editor) should be shown here.");
    // Example: document.getElementById('battle-controls-ui').style.display = 'none';

    clearBattlefield(); // Clear any enemy robots, projectiles, etc.

    if (gameState.ui.gameOverScreen) { // Hide game over screen if it's visible
        gameState.ui.gameOverScreen.style.display = 'none';
    }
    console.log("Switched to ASSEMBLY mode.");
}

})(); // End of IIFE
