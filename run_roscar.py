from lib.roscar import RosCar
import yaml
import os

def load_config():
    """
    LOAD CONFIG FILE
    Convert config.yml to DICT.
    """
    try:
        FileNotFoundError
    except NameError:
        FileNotFoundError = IOError

    cfg = None
    script_dir = os.path.dirname(os.path.realpath(__file__))
    file_path = os.path.join(script_dir, 'config.yml')
    print(file_path)
    if (os.path.isfile(file_path)):
        with open(file_path, 'r') as ymlfile:
            cfg = yaml.safe_load(ymlfile)
    else:
        raise FileNotFoundError(("File not found: config.yml"))
    return cfg

if __name__ == '__main__':
    cfg = load_config()
    roscar = RosCar(cfg)
    roscar.listener()
