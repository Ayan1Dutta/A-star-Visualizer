const gridRows = 20;
const gridCols = 20;

const start = { x: 0, y: 0 };
const goal = { x: 10, y: 15 };
let obstacles = [];

// Directions for movement
const DIRECTIONS = [
    { dx: 0, dy: -1 }, { dx: 0, dy: 1 },
    { dx: -1, dy: 0 }, { dx: 1, dy: 0 },
    { dx: -1, dy: -1 }, { dx: -1, dy: 1 },
    { dx: 1, dy: -1 }, { dx: 1, dy: 1 }
];

// Utility functions
function createGrid() {
    const gridContainer = document.getElementById("grid-container");
    gridContainer.innerHTML = ""; // Clear existing grid

    for (let row = 0; row < gridRows; row++) {
        for (let col = 0; col < gridCols; col++) {
            const cell = document.createElement("div");
            cell.classList.add("cell");
            cell.dataset.row = row;
            cell.dataset.col = col;
            gridContainer.appendChild(cell);
        }
    }

    markCell(start.x, start.y, "start");
    markCell(goal.x, goal.y, "goal");
}

function markCell(x, y, type) {
    const cell = document.querySelector(`.cell[data-row="${y}"][data-col="${x}"]`);
    if (cell) cell.classList.add(type);
}

function clearPath() {
    document.querySelectorAll(".cell.path").forEach(cell => cell.classList.remove("path"));
}

function clearObstacles() {
    document.querySelectorAll(".cell.obstacle").forEach(cell => cell.classList.remove("obstacle"));
    obstacles = [];
}

// Random obstacle generation
function generateRandomObstacles() {
    clearPath();
    clearObstacles();

    const density = 0.25; // 30% obstacle density
    for (let row = 0; row < gridRows; row++) {
        for (let col = 0; col < gridCols; col++) {
            if (Math.random() < density && !(row === start.y && col === start.x) && !(row === goal.y && col === goal.x)) {
                obstacles.push({ x: col, y: row });
                markCell(col, row, "obstacle");
            }
        }
    }
}

function isValid(x, y) {
    return x >= 0 && y >= 0 && x < gridCols && y < gridRows && !obstacles.some(ob => ob.x === x && ob.y === y);
}

// Heuristics
function directionalHeuristic(x, y, px, py, direction) {
    // Movement vector: From the previous cell (parent) to the current cell
    const moveVector = { dx: x - px, dy: y - py };

    // Calculate the magnitude of the directional vector
    const directionMagnitude = Math.sqrt(direction.dx * direction.dx + direction.dy * direction.dy);

    // Normalize the directional vector
    const normalizedDirection = {
        dx: direction.dx / directionMagnitude,
        dy: direction.dy / directionMagnitude,
    };

    // Project the movement vector onto the normalized directional vector
    const projection =
        moveVector.dx * normalizedDirection.dx + moveVector.dy * normalizedDirection.dy;

    // Calculate the perpendicular distance of the current position to the directional vector
    const perpendicularDistance = Math.abs(
        direction.dy * x - direction.dx * y
    ) / directionMagnitude;

    // Combine projection and perpendicular distance into a single heuristic
    // Prioritize reducing perpendicular distance (smaller is better)
    // and maximize projection (larger is better, so we negate it for minimization)
    const weightPerpendicular = 0.6; // Adjust weights as needed
    const weightProjection = 0.4;

    return weightPerpendicular * perpendicularDistance - weightProjection * projection;
}




function octileHeuristic(x, y, px, py, direction) {
    const dx = Math.abs(x - px); // X-distance
    const dy = Math.abs(y - py); // Y-distance

    // Octile distance calculation
    const octileDistance = Math.max(dx, dy) + (Math.sqrt(2) - 1) * Math.min(dx, dy);

    // Perpendicular distance to directional vector (cross product)
    const perpendicularDistance = Math.abs(direction.dy * dx - direction.dx * dy);

    // Combine octile distance with alignment penalty
    return octileDistance + perpendicularDistance * 0.1; // Weight perpendicular distance lower
}


// A* Algorithm
function astar(heuristic, direction = { dx: 0, dy: 0 }) {
    const openSet = [{ x: start.x, y: start.y, g: 0, f: 0 }];
    const cameFrom = new Map();
    const costSoFar = new Map();
    costSoFar.set(`${start.x},${start.y}`, 0);

    while (openSet.length > 0) {
        openSet.sort((a, b) => a.f - b.f); // Sort by lowest f-score
        const current = openSet.shift();

        if (current.x === goal.x && current.y === goal.y) {
            return reconstructPath(cameFrom, current);
        }

        for (const { dx, dy } of DIRECTIONS) {
            const neighborX = current.x + dx;
            const neighborY = current.y + dy;

            if (!isValid(neighborX, neighborY)) continue;

            const newCost = costSoFar.get(`${current.x},${current.y}`) + Math.sqrt(dx * dx + dy * dy);
            if (!costSoFar.has(`${neighborX},${neighborY}`) || newCost < costSoFar.get(`${neighborX},${neighborY}`)) {
                costSoFar.set(`${neighborX},${neighborY}`, newCost);
                const heuristicCost = heuristic(neighborX, neighborY, goal.x, goal.y, direction);
                const f = newCost + heuristicCost;

                openSet.push({ x: neighborX, y: neighborY, g: newCost, f: f });
                cameFrom.set(`${neighborX},${neighborY}`, current);
            }
        }
    }

    return []; // No path found
}

function reconstructPath(cameFrom, current) {
    const path = [];
    while (current) {
        path.unshift({ x: current.x, y: current.y });
        current = cameFrom.get(`${current.x},${current.y}`);
    }
    return path;
}

// Visualization
function visualizePath(heuristicName, heuristic, direction = { dx: 0, dy: 0 }) {
    const path = astar(heuristic, direction);
    if (path.length === 0) {
        alert(`No path found using ${heuristicName} heuristic.`);
        return;
    }
    simulatePath(path);
    console.log(`${heuristicName} Heuristic Path Length:`, path.length - 1);
}

function simulatePath(path) {
    clearPath();
    path.forEach(({ x, y }) => markCell(x, y, "path"));
}

// Event Handlers
document.getElementById("generate-obstacles").addEventListener("click", generateRandomObstacles);

document.getElementById("run-directional").addEventListener("click", () => {
    visualizePath("Directional", directionalHeuristic, { dx: goal.x - start.x, dy: goal.y - start.y });
});

// document.getElementById("run-angular").addEventListener("click", () => {
//     visualizePath("Angular", angularHeuristic, { dx: goal.x - start.x, dy: goal.y - start.y });
// });

document.getElementById("run-octile").addEventListener("click", () => {
    visualizePath("Octile", octileHeuristic);
});

// Initialize
createGrid();
generateRandomObstacles();
