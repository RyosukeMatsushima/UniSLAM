import matplotlib.pyplot as plt
import matplotlib.image as mpimg

class ImageDisplayer:
    def __init__(self, img_path):
        self.img_path = img_path

    def display_image(self):
        try:
            # Load the image
            img = mpimg.imread(self.img_path)

            # Display the image
            plt.imshow(img)
            plt.axis('off')  # Turn off axis labels
            plt.show()

        except FileNotFoundError:
            print(f"Error: Image file '{self.img_path}' not found.")
        except Exception as e:
            print(f"An error occurred: {e}")

# Example usage:
if __name__ == "__main__":

    image_path = './result/edge_gausian_img.jpg'
    img_displayer = ImageDisplayer(image_path)
    img_displayer.display_image()

    image_path = './result/gray_img.jpg'
    img_displayer = ImageDisplayer(image_path)
    img_displayer.display_image()
