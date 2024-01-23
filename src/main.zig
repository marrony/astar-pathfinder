const std = @import("std");
const ray = @cImport({
    @cInclude("raylib.h");
});

const Position = struct {
    x: u16,
    y: u16,
};

const CellKind = enum {
    Empty,
    Wall,
};

const ResultKind = enum {
    NoPath,
    Path,
};

const Cell = struct {
    neighbors: [8]usize = undefined,
    kind: CellKind = undefined,
    fScore: u32 = undefined,
    gScore: u32 = undefined,
    weight: u32 = undefined,
};

const AStar = struct {
    const BitSet = std.DynamicBitSet;
    const InvalidIndex = std.math.maxInt(usize);
    const MaxScore = std.math.maxInt(u32);
    const MaxWeight = 10;

    size: usize,
    grid: []Cell = undefined,
    previous: []usize = undefined,
    openSet: BitSet = undefined, // nodes to be evaluated
    closedSet: BitSet = undefined, // nodes already evaluated

    pub fn init(allocator: std.mem.Allocator, size: usize) !AStar {
        const GridSize = size * size;

        var aStart: AStar = .{
            .size = size,
            .grid = try allocator.alloc(Cell, GridSize),
            .previous = try allocator.alloc(usize, GridSize),
            .openSet = try BitSet.initEmpty(allocator, GridSize),
            .closedSet = try BitSet.initEmpty(allocator, GridSize),
        };

        for (0..GridSize) |i| {
            aStart.grid[i] = .{
                .neighbors = aStart.neighbors(i, false),
                .kind = .Empty,
                .fScore = MaxScore,
                .gScore = MaxScore,
                .weight = 0,
            };
        }

        return aStart;
    }

    pub fn deinit(self: *AStar, allocator: std.mem.Allocator) void {
        allocator.free(self.grid);
        allocator.free(self.previous);
        self.openSet.deinit();
        self.closedSet.deinit();
        self.* = undefined;
    }

    pub fn resize(self: *AStar, allocator: std.mem.Allocator, size: usize) !void {
        if (self.size == size) return;

        self.deinit(allocator);
        self.* = try AStar.init(allocator, size);
    }

    pub fn random(self: *AStar) void {
        const GridSize = self.size * self.size;

        var prng = std.rand.DefaultPrng.init(@intCast(std.time.microTimestamp()));
        var rnd = prng.random();

        for (0..GridSize) |i| {
            self.previous[i] = InvalidIndex;
            self.grid[i].kind = cellKind(rnd);
            self.grid[i].weight = rnd.intRangeLessThan(u32, 1, MaxWeight);
        }
    }

    pub fn position(self: *const AStar, index: usize) Position {
        return .{
            .x = @intCast(index % self.size),
            .y = @intCast(index / self.size),
        };
    }

    pub fn linearIndex(self: *const AStar, pos: Position) usize {
        return pos.y * self.size + pos.x;
    }

    fn offset(self: *const AStar, pos: Position, x: i32, y: i32) usize {
        const ii = @as(i32, @intCast(pos.x)) + x;
        const jj = @as(i32, @intCast(pos.y)) + y;

        if (ii < 0 or ii >= self.size) return InvalidIndex;
        if (jj < 0 or jj >= self.size) return InvalidIndex;

        const out: Position = .{
            .x = @intCast(ii),
            .y = @intCast(jj),
        };

        return self.linearIndex(out);
    }

    fn cellKind(rnd: std.rand.Random) CellKind {
        if (rnd.float(f32) < 0.3)
            return .Wall;

        return .Empty;
    }

    // |0|1|2|
    // |3|x|4|
    // |5|6|7|
    fn neighbors(self: *const AStar, index: usize, diag: bool) [8]usize {
        const pos = self.position(index);

        if (diag) {
            return .{
                self.offset(pos, -1, -1),
                self.offset(pos, 0, -1),
                self.offset(pos, 1, -1),
                self.offset(pos, -1, 0),
                self.offset(pos, 1, 0),
                self.offset(pos, -1, 1),
                self.offset(pos, 0, 1),
                self.offset(pos, 1, 1),
            };
        } else {
            return .{
                self.offset(pos, 0, -1),
                self.offset(pos, -1, 0),
                self.offset(pos, 1, 0),
                self.offset(pos, 0, 1),
                InvalidIndex,
                InvalidIndex,
                InvalidIndex,
                InvalidIndex,
            };
        }
    }

    pub fn changeDiag(self: *AStar, diag: bool) void {
        const GridSize = self.size * self.size;

        for (0..GridSize) |i| {
            self.grid[i].neighbors = self.neighbors(i, diag);
        }
    }

    pub fn updateWeight(self: *AStar) void {
        const GridSize = self.size * self.size;

        var prng = std.rand.DefaultPrng.init(@intCast(std.time.microTimestamp()));
        var rnd = prng.random();

        for (0..GridSize) |i| {
            self.grid[i].weight = rnd.intRangeLessThan(u32, 1, MaxWeight);
        }
    }

    /// squared Euclidian distance
    pub fn heuristic(a: Position, b: Position) u32 {
        const x = @as(i32, @intCast(b.x)) - @as(i32, @intCast(a.x));
        const y = @as(i32, @intCast(b.y)) - @as(i32, @intCast(a.y));

        return @intCast(x * x + y * y);
    }

    fn lowestScore(self: *AStar) usize {
        var iterator = self.openSet.iterator(.{});

        var current = iterator.next().?;
        var lowest_fScore = self.grid[current].fScore;

        while (iterator.next()) |node| {
            if (self.grid[node].kind == .Wall)
                continue;

            //std.log.debug("Lowest {} {} {}", .{ node, self.grid[node].fScore, self.grid[node].gScore });

            if (self.grid[node].fScore < lowest_fScore) {
                current = node;
                lowest_fScore = self.grid[node].fScore;
            }
        }

        return current;
    }

    pub fn find(self: *AStar, start: Position, end: Position) ResultKind {
        const GridSize = self.size * self.size;
        const startIndex = self.linearIndex(start);
        const endIndex = self.linearIndex(end);

        self.openSet.setRangeValue(.{ .start = 0, .end = GridSize }, false);
        self.closedSet.setRangeValue(.{ .start = 0, .end = GridSize }, false);

        self.openSet.set(startIndex);

        for (0..GridSize) |i| {
            self.previous[i] = InvalidIndex;
            self.grid[i].gScore = MaxScore;
            self.grid[i].fScore = MaxScore;
        }

        self.grid[startIndex].gScore = 0;
        self.grid[startIndex].fScore = heuristic(start, end);

        while (self.openSet.count() != 0) {
            const current = self.lowestScore();

            //std.log.debug("Current {}", .{current});

            if (current == endIndex) {
                return .Path;
            }

            self.openSet.unset(current);
            self.closedSet.set(current);

            //std.log.debug("Neighbors = {any}", .{self.grid[current].neighbors});

            for (self.grid[current].neighbors) |neighbor| {
                if (neighbor >= InvalidIndex) continue;
                if (self.closedSet.isSet(neighbor)) continue;
                if (self.grid[neighbor].kind == .Wall) continue;

                const tentative_gScore = self.grid[current].gScore + heuristic(self.position(current), self.position(neighbor)) * self.grid[neighbor].weight;

                //std.log.debug("Neighbor {}: {}", .{ neighbor, tentative_gScore });

                if (!self.openSet.isSet(neighbor)) {
                    self.openSet.set(neighbor);
                } else if (tentative_gScore >= self.grid[neighbor].gScore) {
                    continue;
                }

                self.previous[neighbor] = current;
                self.grid[neighbor].gScore = tentative_gScore;
                self.grid[neighbor].fScore = tentative_gScore + heuristic(self.position(neighbor), end);
            }
        }

        return .NoPath;
    }
};

const WindowWidth = 1400;
const WindowHeight = 1050;
var CellCount: u16 = 16;
var RectWidth: u16 = 0;
var RectHeight: u16 = 0;

var grid: AStar = undefined;

fn drawHelp(font: ray.Font) void {
    const helpTexts = [_][*c]const u8{
        "Help",
        "W = Update grid weights",
        "D = Enable/Disable diagonal",
        "R = Refresh grid",
        "O = Draw open set",
        "C = Draw closed set",
        "P = Draw path",
        "Arrow keys = Move goal",
    };

    const padding = 10;
    const fontSize = 35;
    const h = helpTexts.len * fontSize + 2 * padding;

    var w: f32 = 0;
    for (helpTexts) |text| {
        const m = ray.MeasureTextEx(font, text, fontSize, 1);
        w = @max(w, m.x);
    }

    ray.DrawRectangle(
        @intCast(0),
        @intCast(WindowHeight - h),
        @intFromFloat(w + 2 * padding),
        @intCast(h),
        .{ .r = 0, .g = 0, .b = 0, .a = 200 },
    );

    const y: f32 = WindowHeight - h + padding;

    for (helpTexts, 0..) |text, i| {
        ray.DrawTextEx(
            font,
            text,
            .{
                .x = padding,
                .y = y + @as(f32, @floatFromInt(i)) * fontSize,
            },
            fontSize,
            1,
            ray.WHITE,
        );
    }
}

fn drawRect(index: usize, color: ray.Color) void {
    const pos = grid.position(index);

    ray.DrawRectangle(
        @intCast(@as(u32, pos.x) * RectWidth),
        @intCast(@as(u32, pos.y) * RectHeight),
        @intCast(RectWidth),
        @intCast(RectHeight),
        color,
    );
}

fn drawCell(index: usize, font: ray.Font) void {
    var buff: [32]u8 = undefined;

    const cell = &grid.grid[index];

    if (cell.fScore < AStar.MaxScore) {
        const pos = grid.position(index);

        const x = @as(u32, pos.x) * RectWidth;
        const y = @as(u32, pos.y) * RectHeight;

        const fontSize = 20;

        {
            const text = std.fmt.bufPrintZ(&buff, "{}/{}", .{ cell.fScore, cell.gScore }) catch unreachable;
            const textSize = ray.MeasureTextEx(font, text.ptr, fontSize, 1);

            ray.DrawTextEx(
                font,
                text.ptr,
                .{
                    .x = @as(f32, @floatFromInt(x + RectWidth / 2)) - textSize.x / 2,
                    .y = @as(f32, @floatFromInt(y + RectHeight / 2)) - textSize.y,
                },
                fontSize,
                1,
                ray.BLUE,
            );
        }

        {
            const text = std.fmt.bufPrintZ(&buff, "w={}", .{cell.weight}) catch unreachable;
            const textSize = ray.MeasureTextEx(font, text.ptr, fontSize, 1);

            ray.DrawTextEx(
                font,
                text.ptr,
                .{
                    .x = @as(f32, @floatFromInt(x + RectWidth / 2)) - textSize.x / 2,
                    .y = @as(f32, @floatFromInt(y + RectHeight / 2)) - textSize.y + fontSize,
                },
                fontSize,
                1,
                ray.BLUE,
            );
        }
    }
}

pub fn drawGrid(font: ray.Font, start: Position, end: Position, drawOpenSet: bool, drawClosedSet: bool, drawPath: bool) void {
    if (drawOpenSet) {
        var openSet = grid.openSet.iterator(.{});
        while (openSet.next()) |open| {
            const color: ray.Color = .{
                .r = 255, //@intCast(255 * self.grid[open].weight / 10),
                .g = 255, //@intCast(255 * self.grid[open].weight / 10),
                .b = 0,
                .a = 255,
            };
            drawRect(open, color);
        }
    }

    if (drawClosedSet) {
        var closedSet = grid.closedSet.iterator(.{});
        while (closedSet.next()) |closed| {
            const color: ray.Color = .{
                .r = 255, //@intCast(255 * self.grid[closed].weight / 10),
                .g = 0,
                .b = 0,
                .a = 255,
            };
            drawRect(closed, color);
        }
    }

    if (drawPath) {
        var current = grid.linearIndex(end);

        while (current != AStar.InvalidIndex) {
            const color: ray.Color = .{
                .r = 0,
                .g = 255, //@intCast(255 * self.grid[current].weight / 10),
                .b = 0,
                .a = 255,
            };
            drawRect(current, color);

            current = grid.previous[current];
        }
    }

    const GridSize = grid.size * grid.size;

    for (0..GridSize) |current| {
        const cell = &grid.grid[current];

        if (cell.kind == .Wall) {
            drawRect(current, ray.BLACK);
        } else {
            drawCell(current, font);
        }
    }

    drawRect(grid.linearIndex(start), ray.PINK);
    drawCell(grid.linearIndex(start), font);

    drawRect(grid.linearIndex(end), ray.PINK);
    drawCell(grid.linearIndex(end), font);
}

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer std.debug.assert(gpa.deinit() == .ok);
    const allocator = gpa.allocator();

    ray.InitWindow(WindowWidth, WindowHeight, "A* pathfinding");
    defer ray.CloseWindow();

    ray.SetWindowPosition(0, 0);

    const font = ray.LoadFontEx("fonts/MonacoNerdFont-Regular.ttf", 128, null, 95);
    defer ray.UnloadFont(font);

    var diag = false;
    var drawOpenSet = true;
    var drawClosedSet = true;
    var drawPath = true;

    grid = try AStar.init(allocator, CellCount);
    defer grid.deinit(allocator);

    grid.random();

    const start: Position = .{ .x = 0, .y = 0 };
    var end: Position = .{ .x = CellCount - 1, .y = CellCount - 1 };

    ray.SetTargetFPS(60);
    while (!ray.WindowShouldClose()) {
        if (ray.IsKeyPressed(ray.KEY_LEFT_BRACKET)) {
            CellCount -= 1;
            end = .{ .x = CellCount - 1, .y = CellCount - 1 };

            try grid.resize(allocator, CellCount);
            grid.random();
        }

        if (ray.IsKeyPressed(ray.KEY_RIGHT_BRACKET)) {
            CellCount += 1;
            end = .{ .x = CellCount - 1, .y = CellCount - 1 };

            try grid.resize(allocator, CellCount);
            grid.random();
        }

        RectWidth = WindowWidth / CellCount;
        RectHeight = WindowHeight / CellCount;

        if (ray.IsKeyPressed(ray.KEY_R)) {
            grid.random();
        }

        if (ray.IsKeyPressed(ray.KEY_D)) {
            diag = !diag;
            grid.changeDiag(diag);
        }

        if (ray.IsKeyPressed(ray.KEY_W)) {
            grid.updateWeight();
        }

        if (ray.IsKeyPressed(ray.KEY_O)) {
            drawOpenSet = !drawOpenSet;
        }

        if (ray.IsKeyPressed(ray.KEY_C)) {
            drawClosedSet = !drawClosedSet;
        }

        if (ray.IsKeyPressed(ray.KEY_P)) {
            drawPath = !drawPath;
        }

        if (ray.IsKeyPressed(ray.KEY_UP)) {
            end.y -= 1;
        }

        if (ray.IsKeyPressed(ray.KEY_DOWN)) {
            end.y += 1;
        }

        if (ray.IsKeyPressed(ray.KEY_LEFT)) {
            end.x -= 1;
        }

        if (ray.IsKeyPressed(ray.KEY_RIGHT)) {
            end.x += 1;
        }

        const result = grid.find(start, end);

        ray.BeginDrawing();

        ray.ClearBackground(ray.WHITE);

        drawGrid(font, start, end, drawOpenSet, drawClosedSet, drawPath);

        if (result == .NoPath) {
            const fontSize = 64;
            const text = "No path found";
            const textSize = ray.MeasureTextEx(font, text.ptr, fontSize, 1);

            ray.DrawTextEx(
                font,
                text.ptr,
                .{
                    .x = @as(f32, @floatFromInt(WindowWidth / 2)) - textSize.x / 2,
                    .y = @as(f32, @floatFromInt(WindowHeight / 2)) - textSize.y / 2,
                },
                fontSize,
                1,
                ray.WHITE,
            );
        }

        drawHelp(font);

        // ray.DrawFPS(WindowHeight / 2, 10);

        ray.EndDrawing();
    }
}
