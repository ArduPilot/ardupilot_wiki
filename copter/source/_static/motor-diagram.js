const maxFrames = 64;
let motorsJSON;

async function getJSON(src) {
    const response = await fetch(src);
    const json = await response.json();
    return json;
}

async function initDiagram() {
    motorsJSON = await getJSON('../_static/MotorsJSON.json');
    let notesJSON = await getJSON('../_static/FrameNotes.json');

    motorsJSON.layouts.forEach(function(layout) {
        if (layout.Class in notesJSON) {
            layout.ClassName = notesJSON[layout.Class].Name;
            if (layout.Type in notesJSON[layout.Class]) {
                layout.TypeName = notesJSON[layout.Class][layout.Type].Name;
                if ('Notes' in notesJSON[layout.Class][layout.Type])
                    layout.Notes = notesJSON[layout.Class][layout.Type].Notes;
            }
        }
    });

    const frameClassSelect = document.getElementById('frame-class');
    let frameClasses = Array(maxFrames).fill('');
    motorsJSON.layouts.forEach(function(layout) {
        frameClasses[layout.Class] = layout.ClassName;
    });
    frameClasses.forEach(function(c, i) {
        if (c.length > 0 ) {
            let option = document.createElement('option');
            option.text = i + ': ' + c;
            option.value = i;
            frameClassSelect.add(option);
        }
    });
    onFrameClassChange();
}

function onFrameClassChange() {
    const frameClassSelect = document.getElementById('frame-class');
    const frameTypeSelect = document.getElementById('frame-type');
    frameClass = frameClassSelect.value;
    let frameTypes = Array(maxFrames).fill('');
    motorsJSON.layouts.forEach(function(layout) {
        if (layout.Class == frameClass) frameTypes[layout.Type] = layout.TypeName;
    });
    frameTypeSelect.innerHTML = '';
    frameTypes.forEach(function(t, i) {
        if (t.length > 0 ) {
            let option = document.createElement('option');
            option.text = i + ': ' + t;
            option.value = i;
            frameTypeSelect.add(option);
        }
    });
    generateDiagram();
}

function clearSVGLayers(layerElement) {
    [].slice.call(layerElement.children).forEach(function(layer) {
        layer.innerHTML = '';
    });
}

function appendSVGElement(parent, elementType, x=0, y=0, arg='') {
    const elem = document.createElementNS('http://www.w3.org/2000/svg', elementType);
    elem.setAttribute('x', x);
    elem.setAttribute('y', y);
    if (elementType === 'use') elem.setAttributeNS('http://www.w3.org/1999/xlink', 'href', '#' + arg);
    if (elementType === 'text') elem.textContent = arg;
    parent.appendChild(elem);
    return elem;
}

function appendSVGLine(parent, x1=0, y1=0, x2=0, y2=0) {
    const elem = document.createElementNS('http://www.w3.org/2000/svg', 'line');
    elem.setAttribute('x1', x1);
    elem.setAttribute('y1', y1);
    elem.setAttribute('x2', x2);
    elem.setAttribute('y2', y2);
    parent.appendChild(elem);
    return elem;
}

function getMotorExtents(extents, x, y) {
    function calcMidPoints(points) {
        function midPoint(begin, end) {
            return {'x': (begin.x + end.x) / 2, 'y':  (begin.y + end.y) / 2 };
        }
        points.ForwardMidPt = midPoint(points.ForwardLeft, points.ForwardRight);
        points.BackMidPt = midPoint(points.BackLeft, points.BackRight);
        points.LeftMidPt = midPoint(points.ForwardLeft, points.BackLeft);
        points.RightMidPt = midPoint(points.ForwardRight, points.BackRight);
        return points;
    }
    if (extents === undefined) {
        extents = {
            'ForwardRight' : {'x': 0, 'y': 0},
            'BackRight' : {'x': 0, 'y': 0},
            'BackLeft' : {'x': 0, 'y': 0},
            'ForwardLeft' : {'x': 0, 'y': 0},
            'ForwardMidPt' : {'x': 0, 'y': 0},
            'BackMidPt' : {'x': 0, 'y': 0},
            'LeftMidPt' : {'x': 0, 'y': 0},
            'RightMidPt' : {'x': 0, 'y': 0}
        }
    }
    if (x > extents.ForwardRight.x && y < 0) {
        extents.ForwardRight.x = x;
        extents.ForwardRight.y = y;
        return calcMidPoints(extents);
    }
    if (y < extents.ForwardRight.y && x > 0) {
        extents.ForwardRight.x = x;
        extents.ForwardRight.y = y;
        return calcMidPoints(extents);
    }
    if (x > extents.BackRight.x && y > 0) {
        extents.BackRight.x = x;
        extents.BackRight.y = y;
        return calcMidPoints(extents);
    }
    if (y > extents.BackRight.y && x > 0) {
        extents.BackRight.x = x;
        extents.BackRight.y = y;
        return calcMidPoints(extents);
    }
    if (x < extents.BackLeft.x && y > 0) {
        extents.BackLeft.x = x;
        extents.BackLeft.y = y;
        return calcMidPoints(extents);
    }
    if (y > extents.BackLeft.y && x < 0) {
        extents.BackLeft.x = x;
        extents.BackLeft.y = y;
        return calcMidPoints(extents);
    }
    if (x < extents.ForwardLeft.x && y < 0) {
        extents.ForwardLeft.x = x;
        extents.ForwardLeft.y = y;
        return calcMidPoints(extents);
    }
    if (y < extents.ForwardLeft.y && x < 0) {
        extents.ForwardLeft.x = x;
        extents.ForwardLeft.y = y;
        return calcMidPoints(extents);
    }
    return calcMidPoints(extents);
}

function generateDiagram() {
    const frameClass = document.getElementById('frame-class').value;
    const frameType = document.getElementById('frame-type').value;
    const layout = motorsJSON.layouts.reduce(
        (acc, val) => (val.Class == frameClass && val.Type == frameType) ? val : acc);

    const svg = document.getElementById('motor-diagram')
    const svgLayers = document.getElementById('motor-diagram-layers');
    const layerFrame = document.getElementById('layer-frame');
    const layerMotors = document.getElementById('layer-motors');
    const layerMotorNumbers = document.getElementById('layer-motor-numbers');
    const layerMotorLetters = document.getElementById('layer-motor-letters');
    const layerFrameName = document.getElementById('layer-frame-name');
    const layerFrameNotes = document.getElementById('layer-frame-notes');
    const motorDisplayDiameter = appendSVGElement(layerMotors, 'use', 0, 0, 'CW').getBBox().height + 75;
    clearSVGLayers(svgLayers);

    const uniqueMotorPositions = (function() {
        let arr = [];
        layout.motors.forEach(function(motor) {
            arr.push([Number(motor.Roll), Number(motor.Pitch)]);
        });
        return Array.from(new Map(arr.map((a) => [a.join(), a])).values()).length;
    })();

    // TODO: generate 3D frames for coaxial props
    // TODO: replace procedural generation with image for bi-copter, tri-copter, etc
    const motorDisplayRadius = {
        "2" : motorDisplayDiameter * 1.5,   // (future use?)
        "3" : motorDisplayDiameter * 1.75,
        "4" : motorDisplayDiameter * 1.4,
        "5" : motorDisplayDiameter * 1.5,   // (future use?)
        "6" : motorDisplayDiameter * 1.75,
        "7" : motorDisplayDiameter * 2.25,  // (future use?)
        "8" : motorDisplayDiameter * 2.25,
        "9" : motorDisplayDiameter * 2.33,  // (future use?)
        "10": motorDisplayDiameter * 2.33
    }[uniqueMotorPositions];

    const charCode = 'A'.charCodeAt(0) - 1;
    let motorExtents;
    layout.motors.forEach(function(motor) {
        const θ = Math.atan2(-motor.Pitch, -motor.Roll);
        const r = Math.sqrt(motor.Pitch ** 2 + motor.Roll ** 2) * motorDisplayRadius;
        let x = r * Math.cos(θ);
        let y = r * Math.sin(θ);
        if (motor.Rotation === '?') motor.Rotation = 'NYT';
        appendSVGElement(layerMotors, 'use', x, y, motor.Rotation);
        appendSVGElement(layerMotorNumbers, 'text', x, y, motor.Number);
        if (['V', 'H', 'I'].includes(layout.TypeName)) {
            motorExtents = getMotorExtents(motorExtents, x, y);
        } else {
            appendSVGLine(layerFrame, x, y);
        }

        x = (r + motorDisplayDiameter / 2) * Math.cos(θ);
        y = (r + motorDisplayDiameter / 2) * Math.sin(θ);
        appendSVGElement(layerMotorLetters, 'text', x, y, String.fromCharCode(motor.TestOrder + charCode));
    });

    if (['V', 'H'].includes(layout.TypeName)) {
        appendSVGLine(layerFrame, motorExtents.BackLeft.x, motorExtents.BackLeft.y, motorExtents.ForwardLeft.x, motorExtents.ForwardLeft.y);
        appendSVGLine(layerFrame, motorExtents.BackRight.x, motorExtents.BackRight.y, motorExtents.ForwardRight.x, motorExtents.ForwardRight.y);
        appendSVGLine(layerFrame, motorExtents.LeftMidPt.x, motorExtents.LeftMidPt.y);
        appendSVGLine(layerFrame, motorExtents.RightMidPt.x, motorExtents.RightMidPt.y);
    }
    if (layout.TypeName === 'I') {
        appendSVGLine(layerFrame, motorExtents.BackLeft.x, motorExtents.BackLeft.y, motorExtents.BackRight.x, motorExtents.BackRight.y);
        appendSVGLine(layerFrame, motorExtents.ForwardLeft.x, motorExtents.ForwardLeft.y, motorExtents.ForwardRight.x, motorExtents.ForwardRight.y);
        appendSVGLine(layerFrame, motorExtents.ForwardMidPt.x, motorExtents.ForwardMidPt.y);
        appendSVGLine(layerFrame, motorExtents.BackMidPt.x,  motorExtents.BackMidPt.y);

    }
    appendSVGElement(layerFrame, 'use', 0, 0, 'frame-2d');

    let extents = svg.getBBox();
    let textElem = appendSVGElement(layerFrameName, 'text', 0, 0, [layout.ClassName, layout.TypeName].join(' '));
    let fontSize = parseInt(window.getComputedStyle(textElem).fontSize);
    textElem.setAttribute('y', extents.height + extents.y + fontSize);

    extents = svg.getBBox();
    if ('Notes' in layout) {
        textElem = appendSVGElement(layerFrameNotes, 'text', 0, 0, 'Note: ' + layout.Notes);
        fontSize = parseInt(window.getComputedStyle(textElem).fontSize);
        textElem.setAttribute('y', extents.height + extents.y + fontSize);
    }

    extents = svg.getBBox();
    svg.setAttribute('viewBox', [extents.x, extents.y, extents.width, extents.height].join(' '));
}
