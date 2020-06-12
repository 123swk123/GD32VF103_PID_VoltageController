Import("env")

env.Append(
    CCFLAGS=[
        "-msmall-data-limit=8",
    ],

    LDFLAGS=[
        "-msmall-data-limit=8",
    ]
)
