import time

# My initial thoughts are this: 
# I still need to re-learn the different algorithms, but I do want there to be a global routing -> detailed routing strategy.
# Therefore, I think the main routing function will take in a N x M grid (NOT the placement grid) and use one of the algorithms 
# to find a route between two selected points. By not using the original placement grid here, we will be able to use this same
# function for both global and detailed routing. There will need to be another function that, once a global route is found, transforms that global route
# into its own placement grid that we can input back into this main function and find a detailed route inside of it. 