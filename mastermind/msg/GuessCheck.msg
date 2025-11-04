# Keep track of number of correct colors and their correct positions
# We can just sum the array if we only want to reveal number of correct positions
# Used in /guess_check

int8 num_correct_colors # number of correct colors
int8[4] correct_pos     # [1, 0, 1, 0] signifies 1st and 3rd positions are correct