import math

res = 0.05
width = 640
height = 672
min_x = -22.8
min_y = -10
max_x = min_x + res * width
max_y = min_y + res * height
radius = 50


def read_pgm(path):
    with open(path, "rb") as pgmf:
        '''
        for i in range(2):
            print(pgmf.readline())
        (width, height) = [int(i) for i in pgmf.readline().split()]
        depth = int(pgmf.readline())
        '''
        header = b''
        for i in range(4):
            header += pgmf.readline()

        raster = []
        for y in range(height):
            row = []
            for y in range(width):
                row.append(ord(pgmf.read(1)))
            raster.append(row)
        return header, raster


def write_pgm(path, header, raster):
    with open(path, "wb") as pgmf:
        pgmf.write(header)

        for y in range(height):
            for x in range(width):
                pgmf.write(raster[y][x].to_bytes(1, "big"))


def get_indexes(x, y):
    if min_x <= x <= max_x and min_y <= y <= max_y:
        row = height - int((y - min_y) // res) - 1
        col = int((x - min_x) // res)
        return row, col
    return 0, 0


def write_square_on_raster(raster, row, col):
    for i in range(row - radius, row + radius + 1):
        for j in range(col - radius, col + radius + 1):
            if 0 <= i <= height - 1 and 0 <= j <= width - 1:
                raster[i][j] = 0


def write_circle_on_raster(raster, row, col):
    for i in range(row - radius, row + radius + 1):
        dist = int(math.sqrt(radius ** 2 - abs(i - row) ** 2))
        for j in range(col - dist, col + dist + 1):
            if 0 <= i <= height - 1 and 0 <= j <= width - 1:
                raster[i][j] = 0


header, pgm = read_pgm("C:\\Users\\tomma\\Desktop\\map.pgm")
row, col = get_indexes(8, 0)
write_square_on_raster(pgm, row, col)
row, col = get_indexes(-22, 10)
write_circle_on_raster(pgm, row, col)
write_pgm("C:\\Users\\tomma\\Desktop\\new_map.pgm", header, pgm)
