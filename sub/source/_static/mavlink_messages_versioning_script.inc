<p id="mavlinkVersionSelector" style="display: none;">You can change and check the MAVLink messages for another version:
  <select class="selectpicker" id="mavlinkVersionPicker"></select>
</p>

<script type="text/javascript">
document.addEventListener("DOMContentLoaded", function() {
  // the version pages and their index only exist when the wiki is built with --mavlinkversioning
  fetch("../_static/mavlink-messages-Sub.json")
    .then(function(response) {
      if (!response.ok) { throw new Error(response.status); }
      return response.json();
    })
    .then(function(json) {
      var picker = document.getElementById('mavlinkVersionPicker');
      var current = window.location.pathname.split('/').pop();
      for (var key in json) {
        var opt = document.createElement('option');
        opt.value = json[key];
        opt.innerHTML = key;
        if (json[key] === current) {
          opt.selected = true;
        }
        picker.appendChild(opt);
      }
      picker.addEventListener('change', function() {
        window.location.href = this.value;
      });
      document.getElementById('mavlinkVersionSelector').style.display = '';
    })
    .catch(function() {});
});
</script>
