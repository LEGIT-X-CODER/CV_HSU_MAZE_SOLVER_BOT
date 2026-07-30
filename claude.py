import cv2
import pytesseract
import numpy as np

pytesseract.pytesseract.tesseract_cmd = r"C:\Program Files\Tesseract-OCR\tesseract.exe"

TARGETS = {"H", "S", "U"}
CFG = "--psm 8 -c tessedit_char_whitelist=HSU"

def preprocess(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.bilateralFilter(gray, 9, 75, 75)
    th = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                cv2.THRESH_BINARY_INV, 25, 12)
    return th

def find_char_boxes(th):
    cnts, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    boxes = []
    for c in cnts:
        x, y, w, h = cv2.boundingRect(c)
        if h < 30 or h > 400 or w < 15:
            continue
        if h / w > 4 or w / h > 4:
            continue
        boxes.append((x, y, w, h))
    return boxes

def classify(th, box):
    x, y, w, h = box
    pad = 10
    roi = th[max(0, y-pad):y+h+pad, max(0, x-pad):x+w+pad]
    if roi.size == 0:
        return None
    roi = cv2.resize(roi, (roi.shape[1]*2, roi.shape[0]*2))
    roi = cv2.bitwise_not(roi)  # tesseract wants black text on white
    txt = pytesseract.image_to_string(roi, config=CFG).strip().upper()
    return txt[0] if txt and txt[0] in TARGETS else None

def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Camera not accessible")
        return

    frame_count = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frame_count += 1

        th = preprocess(frame)
        boxes = find_char_boxes(th)

        # classify every 3rd frame to keep it fast
        if frame_count % 3 == 0:
            for box in boxes:
                letter = classify(th, box)
                if letter:
                    x, y, w, h = box
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    cv2.putText(frame, letter, (x, y-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow("HSU Detector", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()