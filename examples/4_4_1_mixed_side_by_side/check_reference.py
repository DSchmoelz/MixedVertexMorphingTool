#####################################################################
# Technische Universitaet Muenchen
# Lehrstuhl für Statik
# Vertex Morphing Tool
# Author: David Schmölz
# david.schmoelz@tum.de
#####################################################################
# Checks reference results with local results
#####################################################################

import numpy as np
import pandas as pd

scaling_types = ["none",  "shape_diag", "shape", "shape_w_off"]

## Check Results
for scaling_type in scaling_types:
    history_folder = f"results/history_scaling_{scaling_type}"
    actual = pd.read_csv(f"{history_folder}/obj_history.csv")
    expected = pd.read_csv(f"references/ref_results/history_scaling_{scaling_type}/obj_history.csv")

    check_okay = pd.testing.assert_frame_equal(expected, actual)
