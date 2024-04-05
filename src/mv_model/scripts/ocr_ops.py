#!/usr/bin/env python3
from azure.ai.vision.imageanalysis.models import VisualFeatures
import json
from azure.ai.vision.imageanalysis import ImageAnalysisClient
from azure.core.credentials import AzureKeyCredential
from dotenv import load_dotenv
import os
load_dotenv()
endpoint = os.environ["VISION_ENDPOINT"]
key = os.environ["VISION_KEY"]
client = ImageAnalysisClient(
    endpoint=endpoint,
    credential=AzureKeyCredential(key)
)
def detect_cert(input_image_byte: bytes) -> json:
        """
        Takes in an image in byte array format, and run OCR on it
        :param input_image_byte: Image byte array []byte
        """

        # https://learn.microsoft.com/en-us/azure/ai-services/computer-vision/how-to/call-analyze-image-40?pivots=programming-language-python
        # First convert the byte to an image source buffer
        result = client.analyze(
            image_data=input_image_byte,
            visual_features=[VisualFeatures.READ]
        )
        print(result)
        # Get the text result
        text_result: result.read =result.read # type: ignore
        threshold = 10

        return text_result

def merge_lines(lines, threshold):
    merged_lines = []
    current_line = None

    # Sort lines based on their y-coordinate
    sorted_lines = sorted(lines, key=lambda x: x['boundingPolygon'][0]['y'])

    for line in sorted_lines:
        if current_line is None:
            current_line = line
        else:
            # Calculate vertical distance between current line and the next line
            distance = line['boundingPolygon'][0]['y'] - current_line['boundingPolygon'][-1]['y']
            if distance <= threshold:
                # Merge the lines
                current_line['text'] += ' ' + line['text']
                current_line['boundingPolygon'].extend(line['boundingPolygon'])
            else:
                # Add the current merged line to the result
                merged_lines.append(current_line)
                current_line = line

    # Add the last merged line to the result
    if current_line is not None:
        merged_lines.append(current_line)

    return merged_lines