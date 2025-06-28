// The Swift Programming Language
// https://docs.swift.org/swift-book


import Foundation
import simd
import apriltag_c
import CoreGraphics


public struct ImageData {
    public let width: UInt32
    public let height: UInt32
    public let stride: UInt32
    public let data: UnsafeBufferPointer<UInt8>
    
    public init(width: UInt32, height: UInt32, stride: UInt32, data: UnsafeBufferPointer<UInt8>) {
        assert(data.count == Int(width) * Int(height))
        self.width = width
        self.height = height
        self.stride = stride
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
    private var _families: [UnsafeMutablePointer<apriltag_family_t>]
    private var _familyTypes: [AprilTagFamily]
    
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
        
        _families = []
        _familyTypes = []
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
            _families.append(f)
            _familyTypes.append(family)
        }
    }
    
    public func processImage(_ image: ImageData) -> [TagDetection] {
        let bufPtr = UnsafeMutablePointer<UInt8>(mutating: image.data.baseAddress!)
        var im = image_u8_t(width: Int32(image.width), height: Int32(image.height), stride: Int32(image.stride), buf: bufPtr)
        return processImageU8(&im)
    }
    
    private func processImageU8(_ image: UnsafeMutablePointer<image_u8_t>) -> [TagDetection] {
        var results = [TagDetection]()
        let detectionsPtr = apriltag_detector_detect(self._detector, image)!
        let detections = detectionsPtr.pointee
        if detections.size == 0 {
            return []
        }
        detections.data.withMemoryRebound(to: UnsafeMutablePointer<apriltag_detection_t>.self, capacity: Int(detections.size)) { pointer in
            for i in 0..<Int(detections.size) {
                let det = pointer.advanced(by: i).pointee.pointee
                if let r = processDetection(det, image: image) {
                    results.append(r)
                }
            }
        }
        apriltag_detections_destroy(detectionsPtr)
        return results
    }
    
    private func processDetection(_ detection: apriltag_detection_t, image: UnsafeMutablePointer<image_u8_t>) -> TagDetection? {
        var det = detection
        let id = Int(det.id)
        guard det.decision_margin > self.decisionMargin else {
            return nil
        }
        guard let tagSize = self.tagIdToSize[id] else {
            return nil
        }
        
        if self.reflectHorizontally {
            det.p.0.0 = Double(image.pointee.width) - det.p.0.0
            det.p.1.0 = Double(image.pointee.width) - det.p.1.0
            det.p.2.0 = Double(image.pointee.width) - det.p.2.0
            det.p.3.0 = Double(image.pointee.width) - det.p.3.0
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
            estimate_tag_pose_orthogonal_iteration(&detInfo, &err1, &pose1, &err2, &pose2, 50)
        }
        let transform = poseToTransform(pose: err1 <= err2 ? pose1 : pose2)
        matd_destroy(pose1.R)
        matd_destroy(pose1.t)
        matd_destroy(pose2.R)
        matd_destroy(pose2.t)
        
        let corners = [
            CGPointMake(det.p.0.0, det.p.0.1),
            CGPointMake(det.p.1.0, det.p.1.1),
            CGPointMake(det.p.2.0, det.p.2.1),
            CGPointMake(det.p.3.0, det.p.3.1)
        ]
        return TagDetection(tagId: id, transform: transform, corners: corners)
    }
    
    deinit {
        apriltag_detector_destroy(_detector);
        for (family, familyType) in zip(_families, _familyTypes) {
            switch familyType {
            case .tag16h5:
                tag16h5_destroy(family)
            case .tag25h9:
                tag25h9_destroy(family)
            case .tag36h10:
                tag36h10_destroy(family)
            case .tag36h11:
                tag36h11_destroy(family)
            case .tagCircle21h7:
                tagCircle21h7_destroy(family)
            case .tagCircle49h12:
                tagCircle49h12_destroy(family)
            case .tagCustom48h12:
                tagCustom48h12_destroy(family)
            case .tagStandard41h12:
                tagStandard41h12_destroy(family)
            case .tagStandard52h13:
                tagStandard52h13_destroy(family)
            }
        }
    }
}

func poseToTransform(pose: apriltag_pose_t) -> simd_float4x4 {
    let R = pose.R
    let t = pose.t
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

