from MapP22 import MapP22
from pathCreator import PathCreator


def main(map_file):
    m = MapP22(map_file)
    p = PathCreator(m,"random")
    essai = p.create_path()
    t = p.path_to_traj(essai)
    m.add_traj(t)
    m.to_JSON(r"C:\Users\Etienne\Documents\Code\MultiAgent\Assignment_2\traj.JSON")
    """
        Magic happens here !
    """



if (__name__ == "__main__"):
    main(r"C:\Users\Etienne\Documents\Code\MultiAgent\Assignment_2\P22.json")