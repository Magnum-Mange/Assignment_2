from MapP22 import MapP22
from pathCreator import PathCreator


def main(map_file):
    m = MapP22(map_file)
    p = PathCreator(m,"genetic")
    for i in range(1):
        essai = p.create_path()
    t = p.path_to_traj(essai)
    print(p.evaluate_paths(essai))
    m.add_traj(t)
    m.to_JSON(r"C:\Users\Etienne\Documents\Code\MultiAgent\Assignment_2\traj.JSON")
    #pa = g.shortest_path(0,31)
    print("Magic")
    """
        Magic happens here !
    """



if (__name__ == "__main__"):
    main(r"C:\Users\Etienne\Documents\Code\MultiAgent\Assignment_2\P22.json")