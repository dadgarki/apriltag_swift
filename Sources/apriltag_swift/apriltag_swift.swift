// The Swift Programming Language
// https://docs.swift.org/swift-book


import Foundation
import simd
import apriltag_c
import CoreGraphics


public struct ImageData {
    public let width: UInt32
    public let height: UInt32
    public let data: UnsafeMutablePointer<UInt8>
    
    public init(width: UInt32, height: UInt32, data: UnsafeMutablePointer<UInt8>) {
        self.width = width
        self.height = height
        self.data = data
    }
}

public struct TagDetection {
    public let tagId: Int
    public let transform: simd_float4x4
    public let corners: [CGPoint]
}

public enum AprilTagFamily {
    case tag16h5
    case tag25h9
    case tag36h10
    case tag36h11
    case tagCircle21h7
    case tagCircle49h12
    case tagCustom48h12
    case tagStandard41h12
    case tagStandard52h13
}

public class AprilTagDetector {
    
    private let _detector: UnsafeMutablePointer<apriltag_detector_t>!
    public var intrinsics: simd_float3x3
    public var tagIdToSize: [Int: Double]
    public var decisionMargin: Float
    public var reflectHorizontally: Bool
    
    public init(intrinsics: simd_float3x3, tagIdToSize: [Int: Double], families: [AprilTagFamily],
                quadDecimate: Float = 1.0, refineEdges: Bool = false,
                decisionMargin: Float = 10.0, reflectHorizontally: Bool = false) {
        
        self.intrinsics = intrinsics
        self.tagIdToSize = tagIdToSize
        self.decisionMargin = decisionMargin
        self.reflectHorizontally = reflectHorizontally
        
        _detector = apriltag_detector_create()
        _detector.pointee.nthreads = 1
        _detector.pointee.quad_decimate = quadDecimate
        _detector.pointee.refine_edges = refineEdges
        _detector.pointee.debug = false
        
        for family in families {
            var f: UnsafeMutablePointer<apriltag_family_t>!
            switch family {
            case .tag16h5:
                f = tag16h5_create()
            case .tag25h9:
                f = tag25h9_create()
            case .tag36h10:
                f = tag36h10_create()
            case .tag36h11:
                f = tag36h11_create()
            case .tagCircle21h7:
                f = tagCircle21h7_create()
            case .tagCircle49h12:
                f = tagCircle49h12_create()
            case .tagCustom48h12:
                f = tagCustom48h12_create()
            case .tagStandard41h12:
                f = tagStandard41h12_create()
            case .tagStandard52h13:
                f = tagStandard52h13_create()
            }
            
            apriltag_detector_add_family(_detector, f)
        }
    }
    
    public func processImage(_ image: ImageData) -> [TagDetection] {
        let im = image_u8_create_stride(image.width, image.height, image.width)
        im?.pointee.buf = image.data
        let results: [TagDetection]
        if let im = im {
            results = processImageU8(im)
        }
        else {
            results = []
        }
        image_u8_destroy(im)
        return results
    }
    
    public func processImageU8(_ image: UnsafeMutablePointer<image_u8_t>) -> [TagDetection] {
        let width = image.pointee.width
        var results = [TagDetection]()
        let detectionsPtr = apriltag_detector_detect(self._detector, image)!
        let detections = detectionsPtr.pointee
        detections.data.withMemoryRebound(to: UnsafeMutablePointer<apriltag_detection_t>.self, capacity: Int(detections.size)) { pointer in
            for i in 0..<Int(detections.size) {
                var det = pointer.advanced(by: i).pointee.pointee
                let id = Int(det.id)
                guard det.decision_margin > self.decisionMargin else {
                    continue
                }
                guard let tagSize = self.tagIdToSize[id] else {
                    continue
                }
                
                if self.reflectHorizontally {
                    det.p.0.0 = Double(width) - det.p.0.0
                    det.p.1.0 = Double(width) - det.p.1.0
                    det.p.2.0 = Double(width) - det.p.2.0
                    det.p.3.0 = Double(width) - det.p.3.0
                }
                var detInfo = apriltag_detection_info_t()
                detInfo.tagsize = tagSize
                detInfo.fx = Double(self.intrinsics.columns.0[0])
                detInfo.fy = Double(self.intrinsics.columns.1[1])
                detInfo.cx = Double(self.intrinsics.columns.2[0])
                detInfo.cy = Double(self.intrinsics.columns.2[1])
                var pose1 = apriltag_pose_t(), pose2 = apriltag_pose_t()
                var err1: Double = 0, err2: Double = 0
                withUnsafeMutablePointer(to: &det) { d in
                    detInfo.det = d
                    estimate_tag_pose_orthogonal_iteration(&detInfo, &err1, &pose1, &err2, &pose2, 10)
                }
                
                let transform = poseToTransform(pose: &pose1)
                let corners = [
                    CGPointMake(det.p.0.0, det.p.0.1),
                    CGPointMake(det.p.1.0, det.p.1.1),
                    CGPointMake(det.p.2.0, det.p.2.1),
                    CGPointMake(det.p.3.0, det.p.3.1)
                ]
                results.append(TagDetection(tagId: id, transform: transform, corners: corners))
            }
        }
        zarray_destroy(detectionsPtr)
        return results
    }
    
    deinit {
        apriltag_detector_destroy(_detector);
    }
}

func poseToTransform(pose: UnsafeMutablePointer<apriltag_pose_t>!) -> simd_float4x4 {
    let R = pose.pointee.R
    let t = pose.pointee.t
    let R00 = Float(matd_get(R, 0, 0))
    let R01 = Float(matd_get(R, 0, 1))
    let R02 = Float(matd_get(R, 0, 2))
    let R10 = Float(matd_get(R, 1, 0))
    let R11 = Float(matd_get(R, 1, 1))
    let R12 = Float(matd_get(R, 1, 2))
    let R20 = Float(matd_get(R, 2, 0))
    let R21 = Float(matd_get(R, 2, 1))
    let R22 = Float(matd_get(R, 2, 2))
    let t0 = Float(t!.pointee.data[0])
    let t1 = Float(t!.pointee.data[1])
    let t2 = Float(t!.pointee.data[2])
    let extrinsics = simd_float4x4(
        simd_float4(R00, R10, R20, 0),
        simd_float4(R01, R11, R21, 0),
        simd_float4(R02, R12, R22, 0),
        simd_float4(t0, t1, t2, 1),
    )
    return simd_mul(
        simd_diagonal_matrix(simd_make_float4(1.0, -1.0, /* invert y */ -1.0 /* invert z */, 1.0)),
        extrinsics)
}

