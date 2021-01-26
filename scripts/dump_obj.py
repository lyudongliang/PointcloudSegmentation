import os


def dump_obj_file(file_path: str, pt_list: list):
    with open(file_path, 'w') as f:
        for pt in pt_list:
            line = 'v ' +  ' '.join(pt) + ' \n'
            f.write(line)
