import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory

def get_file_path(package_name, file_path):
    return os.path.join(
        get_package_share_directory(package_name),
        file_path
    )

def load_file(package_name, file_path):
    absolute_file_path = get_file_path(package_name, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    # parent of IOError, OSError *and* WindowsError where available
    except EnvironmentError:
        return None

def load_yaml(package_name, file_path):
    absolute_file_path = get_file_path(package_name, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    # parent of IOError, OSError *and* WindowsError where available
    except EnvironmentError:
        return None

def save_yaml(data, file_path):
    """
    Safely save a dictionary to a YAML file.
    
    Arguments:
    data -- Dictionary to save to YAML
    file_path -- Path to the YAML file (relative to the package directory)
    """
    # Ensure the directory exists
    dir_path = os.path.dirname(file_path)
    if not os.path.exists(dir_path):
        os.makedirs(dir_path)

    # Get the absolute file path
    #absolute_file_path = get_file_path('', file_path)
    
    try:
        with open(file_path, 'w') as file:
            yaml.safe_dump(
                data,
                file,
                default_flow_style=False,
                allow_unicode=True
            )
        print(f"YAML file successfully saved to {file_path}")
    except Exception as e:
        print(f"Failed to write YAML file: {e}")
