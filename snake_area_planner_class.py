# coding = UTF8
import numpy as np
import math
# from geometry_msgs.msg import Point
import matplotlib.pyplot as plt

class Point():
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.x = x
        self.y = y
        self.z = z
    def print(self):
        return("x = " + str(self.x) + " y = " + str(self.y), " z = " + str(self.z))

class Snake_area_planner():
    def __init__(self, points_of_area: list, end_point_of_path: Point, resolution: float) -> None:
        point_zone_list_round = self._round_point_list(points_of_area, 1)
        self.const_alt = points_of_area[0].z
        self.points_of_area = point_zone_list_round
        self.end_point_of_path = end_point_of_path
        self.resolution = resolution

    # Перезагружаем параметры
    def reload_params(self, points_of_area: list, end_point_of_path: Point, resolution: float) -> None:
        self.points_of_area = points_of_area
        self.end_point_of_path = end_point_of_path
        self.resolution = resolution

    # Получаем траекторию
    def get_trajectory(self):
        # Поворачиваем фигуру
        self.point_group, self.reverse_W, self.rotated_end_path = self._rotator_zone(self.points_of_area, self.end_point_of_path)
        # Вычисляем диапазоны ширины и высоты фигуры
        self.x_range_area, self.y_range_area = self._interval_measurement(self.point_group)
        # Разбиваем вектора, из которых состоит фигура на промежуточные точки по уравнению y = kx + b
        self.points, self.break_grid = self._vector_breaker(self.point_group, self.resolution, self.x_range_area)
        # Сортируем точки, полученые из уравнения y = kx + b в последовательность, соответствующую змейке
        self.point_path, self.vector_path = self._sort_point_list_to_path(self.points, self.break_grid, self.rotated_end_path, self.reverse_W)
        
        return self.point_path, self.vector_path
    
    # Функция преобразования локальных координат в глобальные координаты
    def matrix_of_rotate_2D(self, W, point_of_rotate, point):
        point_out = Point()
        point_out.x = (math.cos(W) * (point_of_rotate.x * math.cos(W) + point_of_rotate.y * math.sin(W))) + (math.sin(W) *
                      (point_of_rotate.x * math.sin(W) - point_of_rotate.y * math.cos(W))) + (point.x * math.cos(W) - point.y * math.sin(W))
        point_out.y = (math.sin(W) * (point_of_rotate.x * math.cos(W) + point_of_rotate.y * math.sin(W))) - (math.cos(W) *
                      (point_of_rotate.x * math.sin(W) - point_of_rotate.y * math.cos(W))) + (point.x * math.sin(W) + point.y * math.cos(W))
        point_out.z = point.z
        
        return point_out

    # Округляем значения
    def _round_point_list(self, point_list: list, accuracy: int) -> list:
        point_list_round = list()
        for point in point_list:
            point_round = Point()
            point_round.x = round(point.x, accuracy)
            point_round.y = round(point.y, accuracy)
            point_round.z = round(point.z, accuracy)
            point_list_round.append(point_round)

        return point_list_round

    # Замеряем крайние границы зоны по осям
    def _interval_measurement(self, group_points: list):
        # находм границы крайних точек фигуры заданной точками
        list_X = list()
        list_Y = list()
        for vector in group_points:
            list_X.append(vector[0].x)
            list_Y.append(vector[0].y)
            list_X.append(vector[1].x)
            list_Y.append(vector[1].y)

        x_range = [np.min(list_X), np.max(list_X)]
        y_range = [np.min(list_Y), np.max(list_Y)]
        return x_range, y_range

    # Группируем точки попарно последовательно 
    def _group_point(self, points_of_area: list):
        groups_point = list()
        for i in range(len(points_of_area) - 1):
            pair = points_of_area[i: i + 2]
            groups_point.append(pair)
        groups_point.append([points_of_area[-1], points_of_area[0]])
        return groups_point

    # Поворачивает исходную зону
    def _rotator_zone(self, points_of_area: list, end_in_path: Point):

        # Группируем точки попарно в последовательность векторов
        groups_point = self._group_point(points_of_area)
        
        # Находим длины всех векторов и записываем в массив для дальнейшего выбора самого большого вектора
        list_r_vectors = list()
        for group in groups_point:
            r = math.sqrt((group[1].x - group[0].x)**2 + (group[1].y - group[0].y)**2)
            list_r_vectors.append(r)
        # Определяем индекс самого наибольшего вектора в массиве groups_point
        index_max_r = np.argmax(list_r_vectors, axis=None)

        most_vector = groups_point[index_max_r]
        new_start_point_index = points_of_area.index(most_vector[0])
        # Составляем новый массив точек с новым началом
        new_points_of_area = points_of_area[new_start_point_index:]
        new_points_of_area = new_points_of_area + points_of_area[:new_start_point_index]
        
        try: 
            # Находим угол, на который смещен самый длинный вектор относительно оси Y 
            cosA = (most_vector[1].y - most_vector[0].y) / math.sqrt((most_vector[1].x - most_vector[0].x)**2 + (most_vector[1].y - most_vector[0].y)**2)
            Alpha = math.acos(cosA)
        except:
            # print("MOST_VECTOR :" + str(most_vector))
            # print("ZERO :" + str((most_vector[1].x - most_vector[0].x)**2) + " " + str((most_vector[1].y - most_vector[0].y)**2))
            return

        if most_vector[1].x - most_vector[0].x > 0.0:
            W = Alpha
        elif most_vector[1].x - most_vector[0].x < 0.0:
            W = -Alpha

        # Поворачиваем нашу фигуру зоны
        rotate_points_of_area= [new_points_of_area[0]]
        for point in new_points_of_area[1:]:
            vector_cords = Point()
            vector_cords.x = point.x - rotate_points_of_area[0].x
            vector_cords.y = point.y - rotate_points_of_area[0].y
            rotate_points_of_area.append(self.matrix_of_rotate_2D(W, rotate_points_of_area[0], vector_cords))
        # Так же поворачиваем точку конца входного маршрута
        vector_cords_end_path = Point()
        vector_cords_end_path.x = end_in_path.x - rotate_points_of_area[0].x
        vector_cords_end_path.y = end_in_path.y - rotate_points_of_area[0].y
        rotated_end_path = self.matrix_of_rotate_2D(W, rotate_points_of_area[0], vector_cords_end_path)

        rotate_points_of_area = self._round_point_list(rotate_points_of_area, 1)
        new_groups_point = self._group_point(rotate_points_of_area)

        return new_groups_point, -W, rotated_end_path

    # Сортируем елементы входного массива по интервалу
    def _list_sort_interval(self, list_interval, interval_list: list):
        if interval_list[1] - interval_list[0] >= 0:
            sorted_list = list()
            for elem in list_interval:
                if elem > interval_list[0] and elem < interval_list[1] or elem == interval_list[0] or elem == interval_list[1]:
                    sorted_list.append(elem)
            return sorted_list
        else:
            print("Не верно задан интервал!")

    # Функция удаляет повторяющиеся точки в списке
    def _del_clone(self, point_list: list) -> list:
        filter_point_list = list()
        filter_point_list.append(point_list[0])
        for point in point_list:
            flag = False
            for filter_point in filter_point_list:
                if point.x == filter_point.x and point.y == filter_point.y:
                    flag = True
            if not flag:
                filter_point_list.append(point)
        return filter_point_list

    # Разбиваем вектора на точки по диапазону X
    def _vector_breaker(self, point_group: list, resolution: float, x_range_area: list):
        point_list = list()
        break_grid = np.arange(x_range_area[0], x_range_area[1], resolution)
        for elem_group in point_group:
            # Условия для разных расположений векторов
            if elem_group[1].x - elem_group[0].x > 0 and not math.sqrt((elem_group[1].x - elem_group[0].x)**2) < 1.0:
                # print("VECTOR_OK")
                # Находим коэффициент k для уравнения прямой y = kx + b
                k = (elem_group[1].y - elem_group[0].y) / \
                    (elem_group[1].x - elem_group[0].x)
                # Находим коэффициент b для уравнения прямой y = kx + b
                point = Point()
                point.y = k*elem_group[0].x
                b = elem_group[0].y - point.y
                # Берем интервалы сетки
                sorted_range = self._list_sort_interval(break_grid, (elem_group[0].x, elem_group[1].x))
                if len(sorted_range) != 0:
                    for x in sorted_range:
                        point = Point()
                        point.x = x
                        point.y = k*x + b
                        point.z = self.const_alt
                        point_list.append(point)
                else:                                                       # Можно выпилить после отладки
                    print("Нет точек для данного разрешения!")

            elif elem_group[1].x - elem_group[0].x < 0: 
                # print("REVERSE_VECTOR")
                # Находим коэффициент k для уравнения прямой y = kx + b
                k = (elem_group[1].y - elem_group[0].y) / \
                    (elem_group[1].x - elem_group[0].x)
                # Находим коэффициент b для уравнения прямой y = kx + b
                point = Point()
                point.y = k*elem_group[0].x
                b = elem_group[0].y - point.y 
                # Берем интервалы сетки
                sorted_range = np.flipud(self._list_sort_interval(break_grid, (elem_group[1].x, elem_group[0].x)))
                if len(sorted_range) != 0:
                    for x in sorted_range:
                        point = Point()
                        point.x = x
                        point.y = k*x + b
                        point.z = self.const_alt
                        point_list.append(point)
                else:                                                       # Можно выпилить после отладки
                    print("Нет точек для данного разрешения!")

            elif math.sqrt(round((elem_group[1].x - elem_group[0].x)**2, 1)) == 0.0 and elem_group[0] not in point_list and elem_group[1] not in point_list:
                # print("VECTOR_NULL")
                elem_group[0].z = self.const_alt
                elem_group[1].z = self.const_alt
                point_list.append(elem_group[0])
                point_list.append(elem_group[1])

        point_list = self._round_point_list(point_list, 1)

        point_list = self._del_clone(point_list)

        return point_list, break_grid

    # Метод для нахождения угла между векторами
    def get_angle_between_vectors(self, first_vector: list, second_vector: list) -> float:
        first_vector_cords = [first_vector[1].x - first_vector[0].x, first_vector[1].y - first_vector[0].y]
        second_vector_cords = [second_vector[1].x - second_vector[0].x, second_vector[1].y - second_vector[0].y]
    
        cosW = ((first_vector_cords[0] * second_vector_cords[0]) + (first_vector_cords[1] * second_vector_cords[1])) / (math.sqrt(first_vector_cords[0]**2 + first_vector_cords[1]**2) * math.sqrt(second_vector_cords[0]**2 + second_vector_cords[1]**2))
    
        W = math.acos(cosW)
        return W

    # Собираем точки в траекторию   (IT'S FINAL)
    def _sort_point_list_to_path(self, point_list: list, break_grid: np.ndarray, last_point: Point, reverse_W: float):

        # = > Сортировка точек в правильную последовательность точек маршрута < = #
        path_list = list()
        # Дополняем список шага по x если среди списка точек
        list_x = list()
        for point in point_list:
            list_x.append(point.x)
        max_x = np.max(list_x)
        
        if max_x not in break_grid and abs(max_x - break_grid[-1]) >= self.resolution:
            break_grid = np.append(break_grid, max_x)

        """
        - определяем все ближайшие точки находящиеся с правого и левого края зоны к концу маршрута из промежуточных точек
        - определяем начальную точку пролета зоны
        """
        path_list = list()
        path_vector_list = list()
        left_edge = break_grid[0]
        right_edge = break_grid[-1]
        r_min = None
        snake_start_point = Point()
        # Определяем с какой точки начнется траектория змейки
        for point in point_list:
            if round(abs(point.x - left_edge), 1) == 0.0 or round(abs(point.x - right_edge), 1) == 0.0:
                r = math.sqrt((point.x - last_point.x) **2 + (point.y - last_point.y)**2)
                if r_min == None:
                    r_min = r
                    snake_start_point = point
                elif r < r_min:
                    r_min = r
                    snake_start_point = point
        
        # Проверяем справа или слева начальная точка зоны
        if round(snake_start_point.x, 1) == round(right_edge, 1):
            break_grid = np.flipud(break_grid)


        last_point = None
        last_vector = None
        #=#=#=#=#=#=#=#=#=#=#=#          # ГЛАВНЫЙ КОСТЫЛЬ НАКОНЕЦ ПЕРЕПИСАНЫЙ В НОРМАЛЬНЫЙ КОД  
        for x in break_grid:
            pair = list()
            if round(math.sqrt((snake_start_point.x - x)**2), 1) == 0.0:
                pair.append(snake_start_point)
                for next_point in point_list:
                    if round(math.sqrt((x - next_point.x)**2), 1) == 0.0 and next_point != snake_start_point:
                        pair.append(next_point)
                
                if len(pair) > 2:   # Сокращаем количество точек до двух
                    pair = sorted(pair, key=lambda point: point.y)
                    pair = [pair[0], pair[-1]]

            elif last_point is not None and last_vector is not None:
                for point in point_list:
                    if round(math.sqrt((point.x - x)**2), 1) == 0.0 and point not in pair:
                        pair.append(point)
                if len(pair) > 2:   # Сокращаем количество точек до двух
                    pair = sorted(pair, key=lambda point: point.y)
                    pair = [pair[0], pair[-1]]

                # Начинаем анализировать вектора
                if len(last_vector) == 1 and len(pair) == 2:
                    first_r = 0.0
                    second_r = 0.0
                    if len(pair) == 2:
                        first_r = math.sqrt((pair[0].x - last_vector[0].x)**2 + (pair[0].y - last_vector[0].y)**2)
                        second_r = math.sqrt((pair[1].x - last_vector[0].x)**2 + (pair[1].y - last_vector[0].y)**2)
                        if first_r > second_r:
                            pair = list(np.flipud(pair))

                # Собираем вектора
                elif len(last_vector) == 2 and len(pair) == 2:
                    W = self.get_angle_between_vectors(pair, last_vector)
                    if W == 0.0:
                        pair = list(np.flipud(pair))
                # elif len(last_vector) == 2 and len(pair) == 1:
                #     print("pass")
                # else:
                #     print("Warning!\n\tlen(last_vector) : " + str(len(last_vector)))
                #     print("\tlen(pair)) : " + str(len(pair)))

            else:
                print("Начальная точка пролета в зону не определена!")
                print("\tround(math.sqrt((snake_start_point.x - x)**2), 1) = " + str(round(math.sqrt((snake_start_point.x - x)**2), 1)))
                continue
            #=#=#=#=#=#=#=#=#=#=#=#

            if len(pair) > 1:          # Выпиливаем нулевыве вектора или же точки без пары
                for point in pair:
                    path_list.append(point)
                path_vector_list.append(pair)
                
                last_point = pair[-1]
                last_vector = pair

        # Обратно поворачиваем нашу фигуру и все посчитанные целевые точки
        rotate_points_of_area = list()
        for point in path_list:
            vector_cords = Point()
            vector_cords.x = point.x - point_list[0].x
            vector_cords.y = point.y - point_list[0].y
            vector_cords.z = point.z
            rotate_points_of_area.append(self.matrix_of_rotate_2D(reverse_W, point_list[0], vector_cords))
        
        rotate_points_of_area = self._round_point_list(rotate_points_of_area, 1)

        rotate_path_vector_list = list()
        for vector in path_vector_list:
            rotate_vector = list()
            for point in vector:
                vector_cords = Point()
                vector_cords.x = point.x - point_list[0].x
                vector_cords.y = point.y - point_list[0].y
                vector_cords.z = point.z
                rotate_vector.append(self.matrix_of_rotate_2D(reverse_W, point_list[0], vector_cords))
                rotate_vector = self._round_point_list(rotate_vector, 1)
            rotate_path_vector_list.append(rotate_vector)

        return rotate_points_of_area, rotate_path_vector_list
