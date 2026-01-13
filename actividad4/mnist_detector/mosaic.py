import torch
import torchvision
import torchvision.transforms as transforms
import matplotlib.pyplot as plt
import numpy as np

def show_pytorch_mosaic(rows=10, cols=10):
    """
    Loads MNIST using PyTorch and displays a mosaic using make_grid.
    """
    # 1. Define Transformations
    # PyTorch requires converting images to Tensors (0-1 range)
    transform = transforms.Compose([transforms.ToTensor()])

    # 2. Load the Dataset
    # download=True checks if data exists; if not, downloads it to ./data
    trainset = torchvision.datasets.MNIST(root='./data', train=True,
                                        download=True, transform=transform)

    # 3. Create a DataLoader
    # We use a batch_size equal to the total images we want to show
    num_images = rows * cols
    dataloader = torch.utils.data.DataLoader(trainset, batch_size=num_images, shuffle=True)

    # 4. Get a single batch of images
    # dataiter returns (images, labels)
    dataiter = iter(dataloader)
    images, labels = next(dataiter)

    # 5. Create the Grid
    # make_grid stitches the batch into a single image tensor
    # padding=0 ensures a seamless "mosaic" look without borders
    mosaic_tensor = torchvision.utils.make_grid(images, nrow=cols, padding=0)

    # 6. Prepare for Plotting
    # PyTorch images are (Channels, Height, Width) -> (1, 280, 280)
    # Matplotlib expects (Height, Width, Channels) -> (280, 280, 1)
    # We use .permute() to rearrange dimensions
    mosaic_np = mosaic_tensor.permute(1, 2, 0).numpy()

    # 7. Display
    plt.figure(figsize=(10, 10))
    plt.imshow(mosaic_np, cmap='gray_r') # Inverted grayscale
    plt.axis('off')
    plt.title(f"PyTorch MNIST Mosaic ({rows}x{cols})")
    plt.show()
    
    print(f"Displayed mosaic of {num_images} images.")

if __name__ == "__main__":
    show_pytorch_mosaic(rows=12, cols=12)
