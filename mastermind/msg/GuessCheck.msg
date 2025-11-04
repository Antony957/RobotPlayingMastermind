# Keep track of which guesses are the correct color and location
# We can just sum each array if we only want to reveal number of correct guesses
# Used in /guess_check

int8[4] correct_colors # [1, 0, 1, 0] signifies 1st and 3rd colors are correct
int8[4] correct_pos    # [1, 0, 1, 0] signifies 1st and 3rd positions are correct