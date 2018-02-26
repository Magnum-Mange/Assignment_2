from MapP22 import MapP22
from pathCreator import PathCreator


def main(map_file):
    map = MapP22(map_file)
    p = PathCreator(map,"random")
    essai = p.create_path()
    p.path_to_traj(essai)
    """
        Magic happens here !
    """



if (__name__ == "__main__"):
    main(r"C:\Users\Etienne\Documents\Code\MultiAgent\Assignment_2\P22.json")