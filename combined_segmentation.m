%%
pyObj = py.importlib.import_module('sutureSegmentation');
py.importlib.reload(pyObj); % python 3

detected_contours = py.contourCalculator.segmented_suture_shape