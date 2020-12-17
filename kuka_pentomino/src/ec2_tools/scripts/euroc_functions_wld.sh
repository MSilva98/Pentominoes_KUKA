#!/bin/bash

# ----------------------------------------------------------------------

putCarriers()
{
    # SLC
    #pi=$(echo '4*a(1)' | bc -l)
    removeCarriers

	gzfactory spawn \
		-f $(rospack find ec2_bringup)/description/models/DLR_SLC/slc-1.4.sdf \
		-x -0.2 -y -2.2 -z 0.91 -Y 1.5 \
		-m carrier_at_wtarget

	gzfactory spawn \
		-f $(rospack find ec2_bringup)/description/models/DLR_SLC/slc-1.4.sdf \
        -x -0.70 -y 3.9 -z 0.91 -Y -1 \
        -m carrier_at_wboard_1
    
	gzfactory spawn \
		-f $(rospack find ec2_bringup)/description/models/DLR_SLC/slc-1.4.sdf \
        -x -0.20 -y 4.0 -z 0.91 -Y -1 \
        -m carrier_at_wboard_2
    
	gzfactory spawn \
		-f $(rospack find ec2_bringup)/description/models/DLR_SLC/slc-1.4.sdf \
        -x 2.45 -y 2.58 -z 0.91 -Y 3.0 \
        -m carrier_at_shelf_1
    
	gzfactory spawn \
		-f $(rospack find ec2_bringup)/description/models/DLR_SLC/slc-1.4.sdf \
        -x 2.45 -y 2.58 -z 1.37 -Y 3.5 \
        -m carrier_at_shelf_2

    #unset pi
}

removeCarriers()
{
    # SLC
    #pi=$(echo '4*a(1)' | bc -l)

	gzfactory delete \
		-m carrier_at_wtarget

	gzfactory delete \
        -m carrier_at_wboard_1
    
	gzfactory delete \
        -m carrier_at_wboard_2
    
	gzfactory delete \
        -m carrier_at_shelf_1
    
	gzfactory delete \
        -m carrier_at_shelf_2

    #unset pi
}
