import os
import cv2
import numpy as np
import pytest
import rclpy
from ros2_sinewave_interfaces.srv import ConvertImage
from ros2_sinewave.sinewave_subscriber import SineWaveSubscriber

@pytest.fixture
def test_node():
    rclpy.init(args=[])
    node = SineWaveSubscriber()
    yield node
    node.destroy_node()
    rclpy.shutdown()

@pytest.fixture
def dummy_image(tmp_path):
    # Create a dummy image
    img = np.zeros((50, 50, 3), dtype=np.uint8)
    file_path = tmp_path / "dummy.jpg"
    cv2.imwrite(str(file_path), img)
    return str(file_path)

def test_call_image_converting_service_success(test_node, dummy_image):
    # Create a fake service request and an empty response
    request = ConvertImage.Request()
    request.image_path = dummy_image
    response = ConvertImage.Response()
    
    response = test_node.call_image_converting_service(request, response)
    
    # Verify that the grayscale image path exists
    assert os.path.exists(response.grayscale_image_path)
    
    # Verify that the image is grayscale
    gray = cv2.imread(response.grayscale_image_path, cv2.IMREAD_GRAYSCALE)
    assert gray is not None
    assert len(gray.shape) == 2  

    # Remove generated grayscale image file
    os.remove(response.grayscale_image_path)

def test_call_image_converting_service_file_not_found(test_node, tmp_path):
    # Create a request with a non-existent image path.
    request = ConvertImage.Request()
    non_existent = tmp_path / "nonexistent.jpg"
    request.image_path = str(non_existent)
    response = ConvertImage.Response()

    response = test_node.call_image_converting_service(request, response)
    
    # Check that the response contains an error message indicating file not found.
    assert "File not found" in response.grayscale_image_path

