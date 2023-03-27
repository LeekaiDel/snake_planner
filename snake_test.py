# coding = utf8
import snake_area_planner_class as snake_planner
from snake_area_planner_class import Point
# from geometry_msgs.msg import Point
import time
import matplotlib.pyplot as plt


class GroupMissionTaskParser():
    def __init__(self):
        self.resolution = 10    # Шаг разбиения фигуры на галсы змейки
        # Последняя точка проложенного маршрута к зоне исследования
        last_point = Point()
        last_point.x = 0.0 #8293.6396484375
        last_point.y = 0.0 #8293.6396484375
        # Список точек обозначающих границу зоны исследования
        # Example 1
        # self.point_list = [
        # Point(x = 8552.283, y = 20136.7, z = 0.0),      # 0
        # Point(x = 8563.144, y = 19746.66, z = 0.0),     # 1
        # Point(x = 8471.484, y = 19576.74, z = 0.0),     # 2
        # Point(x = 8400.625, y = 19713.35, z = 0.0)]     # 4

        # Example 2
        self.point_list = [
        Point(x = 8335.311, y = 19839.37, z = 0.0),   # 0
        Point(x = 8400.625, y = 19713.35, z = 0.0),   # 1
        Point(x = 8552.283, y = 20136.7, z = 0.0),    # 2
        Point(x = 8549.392, y = 20241.7, z = 0.0),    # 3
        Point(x = 8513.392, y = 20242.34, z = 0.0)]   # 4

        # Example 3
        # self.point_list = [
        #     Point(x=8335.311, y=19839.37, z=0.0),   # 0
        #     Point(x=8291.729, y=19923.39, z=0.0),   # 1
        #     Point(x=8343.584, y=20245.29, z=0.0),   # 2
        #     Point(x=8513.392, y=20242.34, z=0.0),   # 3
        #     Point(x=8335.311, y=19839.37, z=0.0)]   # 4

        point_snake_local = list()  # Список точек посчитанной змейки в локальных координатах
        vector_snake_local = list() # Список векторов змейки
        point_zone_list_in_local = self.point_list

        # Создаем объект класса планировщика
        snake_planner_ = snake_planner.Snake_area_planner(point_zone_list_in_local, last_point, self.resolution)
        # Вызываем функцию расчета змейки
        print("Расчет начат.")
        last_time = time.time()
        point_snake_local, vector_snake_local = snake_planner_.get_trajectory()
        current_time = time.time()  
        print("Расчет завершен, затраченное время: " + str(round(current_time - last_time, 3)) + ".")
        
        ### Выводим результат на график ###
        # Рисуем контур зоны исследования
        list_x = list()
        list_y = list()
        for point in self.point_list:
            list_x.append(point.x)
            list_y.append(point.y)

        plt.figure("snake", figsize=(10,10))
        plt.grid(True)
        plt.plot(list_x, list_y)

        # Выводим траекторию змейки на график
        list_x = list()
        list_y = list()
        for point in point_snake_local:
            list_x.append(point.x)
            list_y.append(point.y)

        plt.plot(list_x, list_y)
        plt.show()


def main(args=None):
    mission_task_parser = GroupMissionTaskParser()


if __name__ == '__main__':
    main()
