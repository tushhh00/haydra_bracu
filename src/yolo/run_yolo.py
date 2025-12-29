"""
YOLO v11 Object Detection Script
Uses the trained best.pt model for detection
"""

from ultralytics import YOLO
import cv2
import argparse


def get_device():
    """Get the best available device (CUDA if compatible, else CPU)"""
    import torch
    if torch.cuda.is_available():
        print(f"CUDA available: {torch.cuda.get_device_name(0)}")
        return 'cuda'
    print("CUDA not available, using CPU")
    return 'cpu'


def detect_image(model_path, source, save=True, show=False, conf=0.25):
    """
    Run detection on an image or video
    
    Args:
        model_path: Path to the trained model (best.pt)
        source: Path to image, video, or camera index (0 for webcam)
        save: Save results to runs/detect
        show: Display results in a window
        conf: Confidence threshold
    """
    # Use best available device
    device = get_device()
    print(f"Using device: {device}")
    
    # Load the trained YOLO model
    model = YOLO(model_path)
    model.to(device)
    
    # Run inference
    results = model.predict(
        source=source,
        save=save,
        show=show,
        conf=conf,
        verbose=True
    )
    
    return results


def detect_webcam(model_path, conf=0.25):
    """
    Run real-time detection using webcam
    """
    # Use best available device
    device = get_device()
    print(f"Using device: {device}")
    
    model = YOLO(model_path)
    model.to(device)
    
    # Open webcam
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Could not open webcam")
        return
    
    print("Press 'q' to quit")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Run detection
        results = model.predict(frame, conf=conf, verbose=False)
        
        # Draw results on frame
        annotated_frame = results[0].plot()
        
        # Display
        cv2.imshow("YOLO v11 Detection", annotated_frame)
        
        # Break on 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="YOLO v11 Object Detection")
    parser.add_argument("--model", type=str, default="best.pt", help="Path to model")
    parser.add_argument("--source", type=str, default=None, help="Image/video path or '0' for webcam")
    parser.add_argument("--webcam", action="store_true", help="Use webcam for real-time detection")
    parser.add_argument("--conf", type=float, default=0.25, help="Confidence threshold")
    parser.add_argument("--save", action="store_true", default=True, help="Save results")
    parser.add_argument("--show", action="store_true", help="Show results")
    
    args = parser.parse_args()
    
    if args.webcam:
        # Real-time webcam detection
        detect_webcam(args.model, conf=args.conf)
    elif args.source:
        # Image or video detection
        detect_image(
            model_path=args.model,
            source=args.source,
            save=args.save,
            show=args.show,
            conf=args.conf
        )
    else:
        print("Usage examples:")
        print("  Detect on image:    python run_yolo.py --source image.jpg --show")
        print("  Detect on video:    python run_yolo.py --source video.mp4 --save")
        print("  Detect on webcam:   python run_yolo.py --webcam")
        print("  With custom model:  python run_yolo.py --model best.pt --source image.jpg")
