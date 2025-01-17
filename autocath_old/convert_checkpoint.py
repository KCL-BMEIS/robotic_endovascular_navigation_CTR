import torch

path = "/Users/lennartkarstensen/stacie/AutoCath/example/checkpoint4800089"
checkpoint = torch.load(path)
checkpoint.pop("scheduler_state_dicts")

torch.save(
    checkpoint,
    "/Users/lennartkarstensen/stacie/AutoCath/example/checkpoint4800089_converted",
)

print("success")
