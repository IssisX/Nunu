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
        ui: {
            partsGrid: document.querySelector('.parts-grid'),
            jsonTextarea: document.getElementById('json-textarea'),
            saveButton: document.getElementById('save-button'),
            loadButton: document.getElementById('load-button'),
            clearButton: document.getElementById('clear-button'),
            startBattleButton: document.getElementById('start-battle-button'),
            gameCanvas: document.getElementById('game-canvas'),
            gameTitle: document.querySelector('.game-title'),
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
        // gameState.ui.startBattleButton.addEventListener('click', startBattle);
        console.log("Basic Controls Initialized (placeholders).");
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
        if (gameState.dragging.isDragging) return; // Ignore if already dragging

        const partDef = componentDefaults[partType];
        if (!partDef) return;

        gameState.dragging.isDragging = true;
        gameState.dragging.partType = partType;
        gameState.dragging.originalElement = event.currentTarget; // The .part-icon div

        // Create ghost element
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
        moveGhostElement(event); // Position it initially

        // Add global listeners for mouse move and up
        document.addEventListener('mousemove', handleDragMove);
        document.addEventListener('mouseup', handleDragEnd);

        console.log(`Dragging ${partType} started.`);
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
    }

    /**
     * Handles mouse movement during dragging.
     */
    function handleDragMove(event) {
        if (!gameState.dragging.isDragging) return;

        if (gameState.dragging.isSnapped) {
            // If snapped, the ghost position is controlled by checkSnapPoints
            checkSnapPoints(event); // Re-check to allow un-snapping or snapping to a better point
        } else {
            // If not snapped, move ghost freely with cursor
            moveGhostElement(event);
        }
        checkSnapPoints(event); // Always check for snaps
        // renderSnapHighlights will be called by a render loop if implemented that way
    }


    // Modify handleDragEnd to use snap information
    function handleDragEnd(event) {
        if (!gameState.dragging.isDragging) return;

        let placedPart = null;
        if (gameState.dragging.isSnapped && gameState.dragging.snapTargetBody && gameState.dragging.snapSourcePoint) {
            const targetBody = gameState.dragging.snapTargetBody;
            const targetPointDef = gameState.dragging.snapTargetPoint;
            const sourcePointDef = gameState.dragging.snapSourcePoint;

            // Calculate the position for the new part.
            // The new part's sourcePoint (relative to its own center) should align with targetBody's targetPoint (in world space).
            // New part's angle will be targetBody's angle initially (can be refined)
            const newPartAngle = targetBody.angle;

            // World position of target connection point
            const targetPointWorld = Matter.Vector.add(
                targetBody.position,
                Matter.Vector.rotate({ x: targetPointDef.x, y: targetPointDef.y }, targetBody.angle)
            );

            // The new part's center should be: targetPointWorld - (rotated sourcePointDef by newPartAngle)
            const newPartCenter = Matter.Vector.sub(
                targetPointWorld,
                Matter.Vector.rotate({ x: sourcePointDef.x, y: sourcePointDef.y }, newPartAngle)
            );

            console.log(`Part ${gameState.dragging.partType} snapped and dropped. Target: ${targetBody.customProps.label}, New Part Pos: (${newPartCenter.x.toFixed(0)}, ${newPartCenter.y.toFixed(0)}) Angle: ${newPartAngle.toFixed(2)}`);
            placedPart = createPartOnCanvas(gameState.dragging.partType, newPartCenter.x, newPartCenter.y, newPartAngle, gameState.dragging.snapConstraint);

            if (placedPart && gameState.dragging.snapConstraint) {
                const constraintDef = gameState.dragging.snapConstraint;
                const constraint = Constraint.create({
                    bodyA: constraintDef.bodyA,
                    bodyB: placedPart,
                    pointA: constraintDef.pointA, // Already relative to bodyA center
                    pointB: constraintDef.pointB, // Already relative to bodyB center (sourcePointDef)
                    stiffness: 0.9,
                    length: 0, // Try to keep them at the connection point distance
                    render: {
                        strokeStyle: '#66B2D9', // Accent color for constraint lines
                        lineWidth: 2
                    }
                });
                World.add(gameState.world, constraint);
                gameState.playerRobot.constraints.push(constraint);
                console.log("Constraint created.");

                // Mark connection points as occupied
                targetPointDef.isOccupied = true;
                // The corresponding sourcePointDef on the *instance* of the placedPart also needs marking
                // (This assumes sourcePointDef on componentDefaults is not the one being marked)
                const placedPartSourcePoint = placedPart.customProps.connectionPoints.find(
                    p => p.x === sourcePointDef.x && p.y === sourcePointDef.y && p.type === sourcePointDef.type
                );
                if (placedPartSourcePoint) placedPartSourcePoint.isOccupied = true;

            }

        } else { // Not snapped, regular drop
            const mousePos = getMousePosInCanvas(event);
            const canvasRect = gameState.ui.gameCanvas.getBoundingClientRect();
            const isOverCanvas = event.clientX >= canvasRect.left && event.clientX <= canvasRect.right &&
                                 event.clientY >= canvasRect.top && event.clientY <= canvasRect.bottom;
            if (isOverCanvas) {
                console.log(`Part ${gameState.dragging.partType} dropped at canvas x:${mousePos.x}, y:${mousePos.y}`);
                placedPart = createPartOnCanvas(gameState.dragging.partType, mousePos.x, mousePos.y);
            } else {
                console.log("Part drag cancelled (released outside canvas).");
            }
        }

        // Cleanup drag state
        if (gameState.dragging.ghostElement) {
            gameState.dragging.ghostElement.remove();
        }
        clearSnapState(); // Clear snap-specific states
        gameState.dragging.isDragging = false;
        gameState.dragging.partType = null;
        gameState.dragging.ghostElement = null;
        gameState.dragging.originalElement = null;

        document.removeEventListener('mousemove', handleDragMove);
        document.removeEventListener('mouseup', handleDragEnd);
        // renderSnapHighlights(); // Clear highlights
        console.log("Dragging ended.");
    }

    /**
     * Creates a physical Matter.js part on the canvas.
     * PDS 2.2.1
     */
    function createPartOnCanvas(partType, x, y, angle = 0, snappedTo = null) { // Added snappedTo parameter
        const partDef = componentDefaults[partType];
        if (!partDef) {
            console.error(`Unknown part type: ${partType}`);
            return null;
        }

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

        if (partType === 'chassis') {
            body = Bodies.rectangle(x, y, partDef.width, partDef.height, commonOptions);
        } else if (partType === 'wheel') {
            body = Bodies.circle(x, y, partDef.radius, { ...commonOptions, ...partDef.matterJsOptions });
        } else if (partType === 'spike') {
            // For spike, create a compound body if it has connection points to shift its CoM
            // For now, simple polygon using vertices. Origin (0,0) of vertices is part's center.
            body = Bodies.fromVertices(x, y, Matter.Vertices.create(partDef.vertices), commonOptions);
        } else {
            console.error(`Unsupported part type for creation: ${partType}`);
            return null;
        }

        // Assign custom properties to the Matter.js body
        body.customProps = customProps;

        World.add(gameState.world, body);
        gameState.playerRobot.parts.push(body); // Track player parts

        console.log(`${partDef.label} added to world at (${x.toFixed(0)}, ${y.toFixed(0)}) with ID ${body.customProps.id}. Health: ${body.customProps.health}`);
        return body;
    }

    // PDS 5.2: Game Loop (placeholder, will be expanded)
    Events.on(gameState.engine, 'beforeUpdate', (event) => {
        // This is the main game loop
        // applyPlayerControls(); // If in battle mode
        // applyAIControls(); // If in battle mode
    });

    // PDS 3.3: Collision Handling (placeholder, will be expanded)
    Events.on(gameState.engine, 'collisionStart', (event) => {
        // handleCollisions(event.pairs);
    });

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

    // Start the game once the DOM is fully loaded
    document.addEventListener('DOMContentLoaded', main);

})(); // End of IIFE
