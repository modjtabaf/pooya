#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from scipy.interpolate import interp1d

import helper

def main():
    front_wheel_angle_Rq = helper.load_mat_files_as_bus(
        # '/home/fathi/torc/git/playground/py_ss/data/processed_mat',
        '/home/mojtaba/work/ss_modeling/py_ss/data/processed_mat',
        'front_wheel_angle_Rq')
    for key, val in front_wheel_angle_Rq.items():
        assert isinstance(val, interp1d)
        vname = key.upper()

        hpp_filename = key + '.hpp'
        with open(hpp_filename, 'w') as f:
            f.write('\n')
            f.write('#ifndef __' + vname + '_HPP__\n')
            f.write('#define __' + vname + '_HPP__\n')
            f.write('\n')
            f.write('#include <vector>\n')
            f.write('\n')
            f.write('static constexpr auto %s_LEN = %d;\n' % (vname, val.x.shape[0]))
            f.write('using %s_TYPE = std::vector<double>;\n' % (vname,))
            f.write('\n')
            f.write('extern const %s_TYPE %s_X;\n' % (vname, vname))
            f.write('extern const %s_TYPE %s_Y;\n' % (vname, vname))
            f.write('\n')
            f.write('#endif // __' + key.upper() + '_HPP__\n')

        cpp_filename = key + '.cpp'
        with open(cpp_filename, 'w') as f:
            f.write('\n')
            f.write('#include "%s"\n' % hpp_filename)
            f.write('\n')
            f.write('const %s_TYPE %s_X({\n' % (vname, vname))
            for x in val.x:
                f.write('    %f,\n' % x)
            f.write('});\n')
            f.write('\n')
            f.write('const %s_TYPE %s_Y({\n' % (vname, vname))
            for y in val.y:
                f.write('    %f,\n' % y)
            f.write('});\n')

if __name__ == '__main__':
    main()
