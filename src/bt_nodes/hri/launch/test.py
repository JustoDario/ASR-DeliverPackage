import torch
print(torch.cuda.is_available())         # Debe devolver True
print(torch.cuda.get_device_name(0))     # Te dice el nombre de la GPU
