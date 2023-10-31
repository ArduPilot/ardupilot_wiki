<p>You can change and check the parameters for another version:
  <select class="selectpicker" id="selectPicker"></select>
</p>

<script type="text/javascript">
document.addEventListener("DOMContentLoaded", function() {
  fetch("../_static/parameters-Plane.json")
    .then(function(response) { return response.json(); })
    .then(function(json) {
      appendToSelect(json);
    });

  document.getElementById('selectPicker').addEventListener('change', function() {
    window.location.href = this.value;
  });

  function appendToSelect(json) {
    var selectPicker = document.getElementById('selectPicker');
    for (var key in json) {
      var opt = document.createElement('option');
      opt.value = json[key];
      opt.innerHTML = key;
      selectPicker.appendChild(opt);
    }
  }
});
</script>
