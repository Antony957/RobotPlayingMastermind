# Instruction of LLM part

#### Node llm_pub
```
input: current state, history guess
output: next move

e.g.
input
{
    current_state: ADEG,
    history_guess: [
        ABCD,
        BCDE,
        CDEF
    ]
}

output
{
    success: true,
    next_step: BEFH
}
```