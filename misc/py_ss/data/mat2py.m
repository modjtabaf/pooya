
function mat2py(filename, outpath)
    function traverse(dat, prefix)
        if isstruct(dat)
            names = fieldnames(dat);
            for k = 1:length(names)
                name = names{k};
                disp(name);
                if prefix == ""
                    p = name;
                else
                    p = prefix + "." + name;
                end
                traverse(getfield(dat, name), p);
            end
        elseif isa(dat, 'timeseries')
            data = [dat.Time, dat.Data];
            save(outpath + prefix + ".mat", "data", "-v7");
        else
            disp("Unknown type for " + prefix + "." + "name");
        end
    end

    if nargin == 0
        filename = "kinematics";
    end

    Data = load(filename + ".mat", "-mat");
    traverse(Data, "");
end
