const std = @import("std");
const ray = @cImport({
    @cInclude("raylib.h");
});

fn AStar(comptime Size: usize) type {
    const GridSize = Size * Size;
    const BitSet = std.StaticBitSet(GridSize);
    const InvalidIndex = GridSize;
    const MaxScore = 0xffffffff;
    const MaxWeight = 10;

    return struct {
        grid: [GridSize]Cell = undefined,
        previous: [GridSize]usize = undefined,
        openSet: BitSet = undefined, // nodes to be evaluated
        closedSet: BitSet = undefined, // nodes already evaluated

        const Self = @This();

        const Index = struct {
            x: usize,
            y: usize,

            pub fn init(idx: usize) Index {
                return .{
                    .x = idx % Size,
                    .y = idx / Size,
                };
            }

            pub fn index(idx: Index) usize {
                return idx.y * Size + idx.x;
            }

            pub fn offset(idx: Index, x: i32, y: i32) usize {
                const ii = @as(i32, @intCast(idx.x)) + x;
                const jj = @as(i32, @intCast(idx.y)) + y;

                if (ii < 0 or ii >= Size) return InvalidIndex;
                if (jj < 0 or jj >= Size) return InvalidIndex;

                const out: Index = .{
                    .x = @intCast(ii),
                    .y = @intCast(jj),
                };

                return out.index();
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
            fScore: usize = undefined,
            gScore: usize = undefined,
            weight: usize = undefined,
        };

        fn cellKind(rnd: std.rand.Random) CellKind {
            if (rnd.float(f32) < 0.3)
                return .Wall;

            return .Empty;
        }

        // |0|1|2|
        // |3|x|4|
        // |5|6|7|
        fn neighbors(index: Index, diag: bool) [8]usize {
            if (diag) {
                return .{
                    index.offset(-1, -1),
                    index.offset(0, -1),
                    index.offset(1, -1),
                    index.offset(-1, 0),
                    index.offset(1, 0),
                    index.offset(-1, 1),
                    index.offset(0, 1),
                    index.offset(1, 1),
                };
            } else {
                return .{
                    index.offset(0, -1),
                    index.offset(-1, 0),
                    index.offset(1, 0),
                    index.offset(0, 1),
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
                    .neighbors = neighbors(Index.init(i), diag),
                    .kind = cellKind(rnd.random()),
                    .fScore = MaxScore,
                    .gScore = MaxScore,
                    .weight = rnd.random().intRangeLessThan(usize, 1, MaxWeight),
                };
            }
        }

        pub fn changeDiag(self: *Self, diag: bool) void {
            for (0..GridSize) |i| {
                self.grid[i].neighbors = neighbors(Index.init(i), diag);
            }
        }

        pub fn updateWeight(self: *Self) void {
            var rnd = std.rand.DefaultPrng.init(@intCast(std.time.timestamp()));

            for (0..GridSize) |i| {
                self.grid[i].weight = rnd.random().intRangeLessThan(usize, 1, MaxWeight);
            }
        }

        /// squared Euclidian distance
        pub fn heuristic(a: usize, b: usize) usize {
            const ia = Index.init(a);
            const ib = Index.init(b);

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

        fn drawRect(index: usize, rectWidth: usize, rectHeight: usize, color: ray.Color) void {
            const idx = Index.init(index);

            ray.DrawRectangle(
                @intCast(idx.x * rectWidth),
                @intCast(idx.y * rectHeight),
                @intCast(rectWidth),
                @intCast(rectHeight),
                color,
            );
        }

        fn drawCell(cell: *const Cell, font: ray.Font, idx: usize, rectWidth: usize, rectHeight: usize) void {
            var buff: [32]u8 = undefined;

            if (cell.fScore < MaxScore) {
                const index = Index.init(idx);

                const x = index.x * rectWidth;
                const y = index.y * rectHeight;

                const fontSize = 20;

                {
                    const text = std.fmt.bufPrintZ(&buff, "{}/{}", .{ cell.fScore, cell.gScore }) catch unreachable;
                    const textSize = ray.MeasureTextEx(font, text.ptr, fontSize, 1);

                    ray.DrawTextEx(
                        font,
                        text.ptr,
                        .{
                            .x = @as(f32, @floatFromInt(x + rectWidth / 2)) - textSize.x / 2,
                            .y = @as(f32, @floatFromInt(y + rectHeight / 2)) - textSize.y / 2,
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
                            .x = @as(f32, @floatFromInt(x + rectWidth / 2)) - textSize.x / 2,
                            .y = @as(f32, @floatFromInt(y + rectHeight / 2)) - textSize.y / 2 + fontSize,
                        },
                        fontSize,
                        1,
                        ray.BLUE,
                    );
                }
            }
        }

        pub fn draw(self: *const Self, font: ray.Font, width: u32, height: u32, start: usize, end: usize, drawOpenSet: bool, drawClosedSet: bool, drawPath: bool) void {
            const rectWidth = width / Size;
            const rectHeight = height / Size;

            if (drawOpenSet) {
                var openSet = self.openSet.iterator(.{});
                while (openSet.next()) |open| {
                    const color: ray.Color = .{
                        .r = 255, //@intCast(255 * self.grid[open].weight / 10),
                        .g = 255, //@intCast(255 * self.grid[open].weight / 10),
                        .b = 0,
                        .a = 255,
                    };
                    drawRect(open, rectWidth, rectHeight, color);
                }
            }

            if (drawClosedSet) {
                var closedSet = self.closedSet.iterator(.{});
                while (closedSet.next()) |closed| {
                    const color: ray.Color = .{
                        .r = 255, //@intCast(255 * self.grid[closed].weight / 10),
                        .g = 0,
                        .b = 0,
                        .a = 255,
                    };
                    drawRect(closed, rectWidth, rectHeight, color);
                }
            }

            if (drawPath) {
                var current = end;

                while (current != InvalidIndex) {
                    const color: ray.Color = .{
                        .r = 0,
                        .g = 255, //@intCast(255 * self.grid[current].weight / 10),
                        .b = 0,
                        .a = 255,
                    };
                    drawRect(current, rectWidth, rectHeight, color);

                    current = self.previous[current];
                }
            }

            for (0..GridSize) |current| {
                const cell = &self.grid[current];

                if (cell.kind == .Wall) {
                    drawRect(current, rectWidth, rectHeight, ray.BLACK);
                } else {
                    drawCell(cell, font, current, rectWidth, rectHeight);
                }
            }

            drawRect(start, rectWidth, rectHeight, ray.PINK);
            drawCell(&self.grid[start], font, start, rectWidth, rectHeight);

            drawRect(end, rectWidth, rectHeight, ray.PINK);
            drawCell(&self.grid[end], font, end, rectWidth, rectHeight);
        }
    };
}

const WindowWidth = 1400;
const WindowHeight = 1050;
const CellCount = 15;
const Grid = AStar(CellCount);

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

var grid: Grid = .{};

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

    const start: Grid.Index = .{ .x = 0, .y = 0 };
    var end: Grid.Index = .{ .x = CellCount - 1, .y = CellCount - 1 };

    grid.grid[start.index()].kind = .Empty;
    grid.grid[end.index()].kind = .Empty;

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

        const result = grid.find(start.index(), end.index());

        ray.BeginDrawing();

        ray.ClearBackground(ray.WHITE);

        grid.draw(font, WindowWidth, WindowHeight, start.index(), end.index(), drawOpenSet, drawClosedSet, drawPath);

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
