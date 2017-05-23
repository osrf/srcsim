# Array of times when checkpoints were completed across all tasks.
# If a checkpoint was skipped, its time is zero.
time[] checkpoints_completion

# Array specifying whether checkpoints were restarted. A checkpoint will be
# true if it has been restarted at least once.
bool[] checkpoints_restarted

# The sum of the score across all checkpoints
uint8 score
