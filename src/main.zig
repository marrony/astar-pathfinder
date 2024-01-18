const std = @import("std");
const ray = @cImport({
    @cInclude("raylib.h");
});

fn AStar(comptime Size: u32) type {
    return struct {
        const GridSize = Size * Size;
        const BitSet = std.StaticBitSet(GridSize);
        const InvalidIndex = std.math.maxInt(usize);
        const MaxScore = std.math.maxInt(u32);
        const MaxWeight = 10;

        grid: [GridSize]Cell = undefined,
        previous: [GridSize]usize = undefined,
        openSet: BitSet = undefined, // nodes to be evaluated
        closedSet: BitSet = undefined, // nodes already evaluated

        const Self = @This();

        const Position = struct {
            // this ensures x & y can only hold values between [0, Size-1]
            // ie: if Size=16 then T=u4
            const T = std.math.Log2Int(std.meta.Int(.unsigned, Size));

            x: T,
            y: T,

            pub fn init(index: usize) Position {
                return .{
                    .x = @intCast(index % Size),
                    .y = @intCast(index / Size),
                };
            }

            pub fn linear(pos: Position) usize {
                return pos.y * Size + pos.x;
            }

            pub fn offset(pos: Position, x: i32, y: i32) usize {
                const ii = @as(i32, @intCast(pos.x)) + x;
                const jj = @as(i32, @intCast(pos.y)) + y;

                if (ii < 0 or ii >= Size) return InvalidIndex;
                if (jj < 0 or jj >= Size) return InvalidIndex;

                const out: Position = .{
                    .x = @intCast(ii),
                    .y = @intCast(jj),
                };

                return out.linear();
            }
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

        fn cellKind(rnd: std.rand.Random) CellKind {
            if (rnd.float(f32) < 0.3)
                return .Wall;

            return .Empty;
        }

        // |0|1|2|
        // |3|x|4|
        // |5|6|7|
        fn neighbors(pos: Position, diag: bool) [8]usize {
            if (diag) {
                return .{
                    pos.offset(-1, -1),
                    pos.offset(0, -1),
                    pos.offset(1, -1),
                    pos.offset(-1, 0),
                    pos.offset(1, 0),
                    pos.offset(-1, 1),
                    pos.offset(0, 1),
                    pos.offset(1, 1),
                };
            } else {
                return .{
                    pos.offset(0, -1),
                    pos.offset(-1, 0),
                    pos.offset(1, 0),
                    pos.offset(0, 1),
                    InvalidIndex,
                    InvalidIndex,
                    InvalidIndex,
                    InvalidIndex,
                };
            }
        }

        pub fn reset(self: *Self, diag: bool) void {
            var rnd = std.rand.DefaultPrng.init(@intCast(std.time.timestamp()));

            for (0..GridSize) |i| {
                self.previous[i] = InvalidIndex;
                self.grid[i] = .{
                    .neighbors = neighbors(Position.init(i), diag),
                    .kind = cellKind(rnd.random()),
                    .fScore = MaxScore,
                    .gScore = MaxScore,
                    .weight = rnd.random().intRangeLessThan(u32, 1, MaxWeight),
                };
            }
        }

        pub fn changeDiag(self: *Self, diag: bool) void {
            for (0..GridSize) |i| {
                self.grid[i].neighbors = neighbors(Position.init(i), diag);
            }
        }

        pub fn updateWeight(self: *Self) void {
            var rnd = std.rand.DefaultPrng.init(@intCast(std.time.timestamp()));

            for (0..GridSize) |i| {
                self.grid[i].weight = rnd.random().intRangeLessThan(u32, 1, MaxWeight);
            }
        }

        /// squared Euclidian distance
        pub fn heuristic(a: usize, b: usize) u32 {
            const ia = Position.init(a);
            const ib = Position.init(b);

            const x = @as(i32, @intCast(ib.x)) - @as(i32, @intCast(ia.x));
            const y = @as(i32, @intCast(ib.y)) - @as(i32, @intCast(ia.y));

            return @intCast(x * x + y * y);
        }

        fn lowestScore(self: *Self) usize {
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

        pub fn find(self: *Self, start: usize, end: usize) ResultKind {
            self.openSet = BitSet.initEmpty();
            self.closedSet = BitSet.initEmpty();

            self.openSet.set(start);

            for (0..GridSize) |i| {
                self.previous[i] = InvalidIndex;
                self.grid[i].gScore = MaxScore;
                self.grid[i].fScore = MaxScore;
            }

            self.grid[start].gScore = 0;
            self.grid[start].fScore = heuristic(start, end);

            while (self.openSet.count() != 0) {
                const current = self.lowestScore();

                //std.log.debug("Current {}", .{current});

                if (current == end) {
                    return .Path;
                }

                self.openSet.unset(current);
                self.closedSet.set(current);

                //std.log.debug("Neighbors = {any}", .{self.grid[current].neighbors});

                for (self.grid[current].neighbors) |neighbor| {
                    if (neighbor >= InvalidIndex) continue;
                    if (self.closedSet.isSet(neighbor)) continue;
                    if (self.grid[neighbor].kind == .Wall) continue;

                    const tentative_gScore = self.grid[current].gScore + heuristic(current, neighbor) * self.grid[neighbor].weight;

                    //std.log.debug("Neighbor {}: {}", .{ neighbor, tentative_gScore });

                    if (!self.openSet.isSet(neighbor)) {
                        self.openSet.set(neighbor);
                    } else if (tentative_gScore >= self.grid[neighbor].gScore) {
                        continue;
                    }

                    self.previous[neighbor] = current;
                    self.grid[neighbor].gScore = tentative_gScore;
                    self.grid[neighbor].fScore = tentative_gScore + heuristic(neighbor, end);
                }
            }

            return .NoPath;
        }
    };
}

const WindowWidth = 1400;
const WindowHeight = 1050;
const CellCount = 16;
const RectWidth = WindowWidth / CellCount;
const RectHeight = WindowHeight / CellCount;
const Grid = AStar(CellCount);
var grid: Grid = .{};

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
    const pos = Grid.Position.init(index);

    ray.DrawRectangle(
        @intCast(@as(u32, pos.x) * RectWidth),
        @intCast(@as(u32, pos.y) * RectHeight),
        @intCast(RectWidth),
        @intCast(RectHeight),
        color,
    );
}

fn drawCell(cell: *const Grid.Cell, font: ray.Font, index: usize) void {
    var buff: [32]u8 = undefined;

    if (cell.fScore < Grid.MaxScore) {
        const pos = Grid.Position.init(index);

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

pub fn drawGrid(font: ray.Font, start: usize, end: usize, drawOpenSet: bool, drawClosedSet: bool, drawPath: bool) void {
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
        var current = end;

        while (current != Grid.InvalidIndex) {
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

    for (0..Grid.GridSize) |current| {
        const cell = &grid.grid[current];

        if (cell.kind == .Wall) {
            drawRect(current, ray.BLACK);
        } else {
            drawCell(cell, font, current);
        }
    }

    drawRect(start, ray.PINK);
    drawCell(&grid.grid[start], font, start);

    drawRect(end, ray.PINK);
    drawCell(&grid.grid[end], font, end);
}

pub fn main() !void {
    ray.InitWindow(WindowWidth, WindowHeight, "A* pathfinding");
    defer ray.CloseWindow();

    ray.SetWindowPosition(0, 0);

    const font = ray.LoadFontEx("fonts/MonacoNerdFont-Regular.ttf", 128, null, 95);
    defer ray.UnloadFont(font);

    var diag = false;
    var drawOpenSet = true;
    var drawClosedSet = true;
    var drawPath = true;

    grid.reset(diag);

    const start: Grid.Position = .{ .x = 0, .y = 0 };
    var end: Grid.Position = .{ .x = CellCount - 1, .y = CellCount - 1 };

    grid.grid[start.linear()].kind = .Empty;
    grid.grid[end.linear()].kind = .Empty;

    ray.SetTargetFPS(60);
    while (!ray.WindowShouldClose()) {
        if (ray.IsKeyPressed(ray.KEY_D)) {
            diag = !diag;
            grid.changeDiag(diag);
        }

        if (ray.IsKeyPressed(ray.KEY_R)) {
            grid.reset(diag);
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

        const result = grid.find(start.linear(), end.linear());

        ray.BeginDrawing();

        ray.ClearBackground(ray.WHITE);

        drawGrid(font, start.linear(), end.linear(), drawOpenSet, drawClosedSet, drawPath);

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
